/**
 * Copyright (c) 2016 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * RFC 3550: RTP: A Transport Protocol for Real-Time Applications
 *
 * 6. RTP Control Protocol -- RTCP
 */

#include "rtp_priv.h"

/* The max length in bits of the 'useful data' in a packet chunk. */
#define RUN_LENGTH_CHUNK_ACK_LG 13
#define STATUS_VECTOR_CHUNK_ACK_LG 14
#define STATUS_VECTOR_TWO_BIT_SYMBOLS_MASK 0x4000
#define STATUS_VECTOR_CHUNK_MASK 0x8000

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */


static int rtcp_pkt_write_header(struct pomp_buffer *buf,
				 size_t *pos,
				 const struct rtcp_pkt_header *header)
{
	int res = 0;
	CHECK(rtp_write_u8(buf, pos, header->flags));
	CHECK(rtp_write_u8(buf, pos, header->type));
	CHECK(rtp_write_u16(buf, pos, header->len));
out:
	return res;
}


/**
 * 6.4 Sender and Receiver Reports
 * 6.4.1 SR: Sender Report RTCP Packet
 * 6.4.2 RR: Receiver Report RTCP Packet
 */
static int rtcp_pkt_write_report_block(struct pomp_buffer *buf,
				       size_t *pos,
				       const struct rtcp_pkt_report_block *rb)
{
	int res = 0;
	uint32_t fraction_lost = (rb->fraction << 24) | (rb->lost & 0xffffff);
	CHECK(rtp_write_u32(buf, pos, rb->ssrc));
	CHECK(rtp_write_u32(buf, pos, fraction_lost));
	CHECK(rtp_write_u32(buf, pos, rb->ext_highest_seqnum));
	CHECK(rtp_write_u32(buf, pos, rb->jitter));
	CHECK(rtp_write_u16(buf, pos, rb->lsr.seconds));
	CHECK(rtp_write_u16(buf, pos, rb->lsr.fraction));
	CHECK(rtp_write_u32(buf, pos, rb->dlsr));
out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 * 6.5.8 PRIV: Private Extensions SDES Item
 */
static int rtcp_pkt_write_sdes_item(struct pomp_buffer *buf,
				    size_t *pos,
				    const struct rtcp_pkt_sdes_item *item)
{
	int res = 0;
	uint32_t data_len = 0;

	CHECK(rtp_write_u8(buf, pos, item->type));

	if (item->data != NULL && item->data_len > 0) {
		/* Normal item data */
		CHECK(rtp_write_u8(buf, pos, item->data_len));
		CHECK(pomp_buffer_write(buf, pos, item->data, item->data_len));
	} else if (item->type == RTCP_PKT_SDES_TYPE_PRIV) {
		/* Private item data: prefix_len + prefix + value */
		data_len = item->priv.prefix_len + item->priv.value_len + 1;
		if (data_len > 255) {
			res = -EINVAL;
			ULOGE("sdes: bad prefix/value length: %u/%u",
			      item->priv.prefix_len,
			      item->priv.value_len);
			goto out;
		}
		CHECK(rtp_write_u8(buf, pos, (uint8_t)data_len));
		CHECK(rtp_write_u8(buf, pos, item->priv.prefix_len));
		CHECK(pomp_buffer_write(
			buf, pos, item->priv.prefix, item->priv.prefix_len));
		CHECK(pomp_buffer_write(
			buf, pos, item->priv.value, item->priv.value_len));
	} else {
		/* No item data */
		CHECK(rtp_write_u8(buf, pos, 0));
	}

out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 */
static int rtcp_pkt_write_sdes_chunk(struct pomp_buffer *buf,
				     size_t *pos,
				     const struct rtcp_pkt_sdes_chunk *chunk)
{
	int res = 0;

	CHECK(rtp_write_u32(buf, pos, chunk->ssrc));

	/* Write items */
	for (uint32_t i = 0; i < chunk->item_count; i++)
		CHECK(rtcp_pkt_write_sdes_item(buf, pos, &chunk->items[i]));

	/* Add final null item and pad until aligned */
	CHECK(rtp_write_u8(buf, pos, 0));
	while (*pos % 4 != 0)
		CHECK(rtp_write_u8(buf, pos, 0));

out:
	return res;
}


/**
 * Get the RTCP Feedback packet chunks 'status'.
 * This means:
 *	- symbol list in Status Vector Chunk
 *	- run length in Run Length Chunk
 * Refer to google congestion control draft for more info:
 * draft-holmer-rmcat-transport-wide-cc-extensions-01.pdf
 */
static uint16_t
build_rtcpfb_chunk_status(uint8_t *symbols, uint16_t len, bool two_bit_symbols)
{
	const int bits_per_symbol = two_bit_symbols ? 2 : 1;
	const int nb_symbols = STATUS_VECTOR_CHUNK_ACK_LG / bits_per_symbol;

	uint16_t status = STATUS_VECTOR_CHUNK_MASK;
	if (two_bit_symbols)
		status |= STATUS_VECTOR_TWO_BIT_SYMBOLS_MASK;

	for (int i = 0; i < len; i++) {
		const int bit_pos = (nb_symbols - 1 - i) * bits_per_symbol;
		status |= symbols[i] << bit_pos;
	}

	return status;
}


const char *rtcp_pkt_type_str(uint8_t type)
{
	switch (type) {
	case RTCP_PKT_TYPE_SR:
		return "RR";
	case RTCP_PKT_TYPE_RR:
		return "SR";
	case RTCP_PKT_TYPE_SDES:
		return "SDES";
	case RTCP_PKT_TYPE_BYE:
		return "BYE";
	case RTCP_PKT_TYPE_APP:
		return "APP";
	case RTCP_PKT_TYPE_RTPFB:
		return "RTPFB";
	default:
		return "UNKNOWN";
	}
}


const char *rtcp_pkt_sdes_type_str(uint8_t type)
{
	switch (type) {
	case RTCP_PKT_SDES_TYPE_END:
		return "END";
	case RTCP_PKT_SDES_TYPE_CNAME:
		return "CNAME";
	case RTCP_PKT_SDES_TYPE_NAME:
		return "NAME";
	case RTCP_PKT_SDES_TYPE_EMAIL:
		return "EMAIL";
	case RTCP_PKT_SDES_TYPE_PHONE:
		return "PHONE";
	case RTCP_PKT_SDES_TYPE_LOC:
		return "LOC";
	case RTCP_PKT_SDES_TYPE_TOOL:
		return "TOOL";
	case RTCP_PKT_SDES_TYPE_NOTE:
		return "NOTE";
	case RTCP_PKT_SDES_TYPE_PRIV:
		return "PRIV";
	default:
		return "UNKNOWN";
	}
}


/**
 * 6.4.1 SR: Sender Report RTCP Packet
 */
int rtcp_pkt_write_sender_report(struct pomp_buffer *buf,
				 size_t *pos,
				 const struct rtcp_pkt_sender_report *sr)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sr == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	CHECK(rtp_write_u32(buf, pos, sr->ssrc));
	CHECK(rtp_write_u32(buf, pos, sr->ntp_timestamp.seconds));
	CHECK(rtp_write_u32(buf, pos, sr->ntp_timestamp.fraction));
	CHECK(rtp_write_u32(buf, pos, sr->rtp_timestamp));
	CHECK(rtp_write_u32(buf, pos, sr->sender_packet_count));
	CHECK(rtp_write_u32(buf, pos, sr->sender_byte_count));

	for (uint32_t i = 0; i < sr->report_count; i++)
		CHECK(rtcp_pkt_write_report_block(buf, pos, &sr->reports[i]));

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(sr->report_count << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_SR;
	header.len = (*pos - header_pos) / 4 - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


/**
 * 6.4.2 RR: Receiver Report RTCP Packet
 */
int rtcp_pkt_write_receiver_report(struct pomp_buffer *buf,
				   size_t *pos,
				   const struct rtcp_pkt_receiver_report *rr)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(rr == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	CHECK(rtp_write_u32(buf, pos, rr->ssrc));

	for (uint32_t i = 0; i < rr->report_count; i++)
		CHECK(rtcp_pkt_write_report_block(buf, pos, &rr->reports[i]));

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(rr->report_count << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_RR;
	header.len = (*pos - header_pos) / 4 - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 */
int rtcp_pkt_write_sdes(struct pomp_buffer *buf,
			size_t *pos,
			const struct rtcp_pkt_sdes *sdes)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(sdes == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	/* Write chunks */
	for (uint32_t i = 0; i < sdes->chunk_count; i++)
		CHECK(rtcp_pkt_write_sdes_chunk(buf, pos, &sdes->chunks[i]));

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(sdes->chunk_count << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_SDES;
	header.len = (*pos - header_pos) / 4 - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


/**
 * 6.6 BYE: Goodbye RTCP Packet
 */
int rtcp_pkt_write_bye(struct pomp_buffer *buf,
		       size_t *pos,
		       const struct rtcp_pkt_bye *bye)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(bye == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	/* Write sources */
	for (uint32_t i = 0; i < bye->source_count; i++)
		CHECK(rtp_write_u32(buf, pos, bye->sources[i]));

	/* Write reason */
	if (bye->reason != NULL && bye->reason_len > 0) {
		CHECK(rtp_write_u8(buf, pos, bye->reason_len));
		CHECK(pomp_buffer_write(
			buf, pos, bye->reason, bye->reason_len));
		/* Pad until aligned */
		while (*pos % 4 != 0)
			CHECK(rtp_write_u8(buf, pos, 0));
	}

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(bye->source_count << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_BYE;
	header.len = (*pos - header_pos) / 4 - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


/**
 * 6.7 APP: Application-Defined RTCP Packet
 */
int rtcp_pkt_write_app(struct pomp_buffer *buf,
		       size_t *pos,
		       const struct rtcp_pkt_app *app)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(app == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	/* Write data */
	CHECK(rtp_write_u32(buf, pos, app->ssrc));
	CHECK(rtp_write_u32(buf, pos, app->name));
	if (app->data != NULL && app->data_len > 0)
		CHECK(pomp_buffer_write(buf, pos, app->data, app->data_len));

	/* Align on 32-bit */
	while (*pos % 4 != 0)
		CHECK(rtp_write_u8(buf, pos, 0));

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(app->subtype << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_APP;
	header.len = (*pos - header_pos) / 4 - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


/**
 * RTPFB Congestion Control Feedback
 */
int rtcp_pkt_write_rtpfb(struct pomp_buffer *buf,
			 size_t *pos,
			 const struct rtcp_pkt_rtpfb_report *rtpfb)
{
	int res = 0;
	struct rtcp_pkt_header header;
	size_t header_pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pos == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(rtpfb == NULL, EINVAL);

	/* Remember where to write header and skip it */
	header_pos = *pos;
	*pos += RTCP_PKT_HEADER_SIZE;

	CHECK(rtp_write_u32(buf, pos, rtpfb->sender_ssrc));
	CHECK(rtp_write_u32(buf, pos, rtpfb->media_ssrc));
	CHECK(rtp_write_u16(buf, pos, rtpfb->base_seq));
	CHECK(rtp_write_u16(buf, pos, rtpfb->status_count));

	CHECK(rtp_write_u16(buf, pos, rtpfb->ref_time >> 8));
	CHECK(rtp_write_u8(buf, pos, rtpfb->ref_time & 0xFF));
	CHECK(rtp_write_u8(buf, pos, rtpfb->feedback_pkt_count));

	/* Write packet chunks */
	struct pkt_chunk {
		/* Represents the Run Length in a Run Length Chunk.
		 * Represents Status symbol list in Status Vector Chunk. */
		uint8_t ack[STATUS_VECTOR_CHUNK_ACK_LG];
		/* Boolean value to indicate if it is
		 * a run length chunk or a status chunk */
		uint8_t run;
		/* Boolean value to indicate if it is a status with
		 * only small deltas or also contains large deltas */
		uint8_t large;
		uint16_t pos;
	};

	uint16_t i, j;
	struct pkt_chunk chunk;
	chunk.run = 1;
	chunk.large = 0;
	chunk.pos = 0;
	for (i = 0; i < rtpfb->status_count; ++i) {
		/* Check if chunk should be written */
		if (chunk.pos == STATUS_VECTOR_CHUNK_ACK_LG &&
		    chunk.large == 0 && chunk.run == 0) {
			/* Status Vector Chunk containing 14 1-bits symbols */
			uint16_t status = build_rtcpfb_chunk_status(
				chunk.ack, chunk.pos, false);
			CHECK(rtp_write_u16(buf, pos, status));
			chunk.pos = 0;
		} else if (chunk.pos >= 7 && chunk.large && chunk.run == 0) {
			/* Status Vector Chunk containing 7 2-bits symbols */
			uint16_t status = build_rtcpfb_chunk_status(
				chunk.ack, chunk.pos, true);
			CHECK(rtp_write_u16(buf, pos, status));
			uint8_t left = chunk.pos - 7;
			for (j = 0; j < left; ++j)
				chunk.ack[j] = chunk.ack[j + 7];
			chunk.pos = left;
		} else if (chunk.pos >= STATUS_VECTOR_CHUNK_ACK_LG &&
			   chunk.ack[0] !=
				   rtpfb->feedbacks[i].pkt_status_symbol) {
			uint16_t status =
				(chunk.ack[0] << RUN_LENGTH_CHUNK_ACK_LG) |
				chunk.pos;
			CHECK(rtp_write_u16(buf, pos, status));
			chunk.pos = 0;
		}
		if (chunk.pos == 0) {
			chunk.large = 0;
			chunk.run = 1;
		}

		if (chunk.pos < STATUS_VECTOR_CHUNK_ACK_LG) {
			chunk.ack[chunk.pos] =
				rtpfb->feedbacks[i].pkt_status_symbol;
			if (rtpfb->feedbacks[i].pkt_status_symbol > 1)
				chunk.large = 1;
			if (chunk.pos > 0 &&
			    chunk.ack[chunk.pos - 1] != chunk.ack[chunk.pos])
				chunk.run = 0;
		}
		chunk.pos++;
	}

	if (chunk.pos != 0) {
		if (chunk.run) {
			uint16_t status =
				(chunk.ack[0] << RUN_LENGTH_CHUNK_ACK_LG) |
				chunk.pos;
			CHECK(rtp_write_u16(buf, pos, status));
		} else if (chunk.large) {
			/* Status Vector Chunk containing 7 2-bits symbols with
			 * long delta */

			/* Write a first chunk if needed */
			if (chunk.pos >= 7) {
				uint16_t status = build_rtcpfb_chunk_status(
					chunk.ack, chunk.pos, true);
				CHECK(rtp_write_u16(buf, pos, status));
				uint8_t left = chunk.pos - 7;
				for (j = 0; j < left; ++j)
					chunk.ack[j] = chunk.ack[j + 7];
				chunk.pos = left;
			}
			if (chunk.pos != 0) {
				uint16_t status = build_rtcpfb_chunk_status(
					chunk.ack, chunk.pos, true);
				CHECK(rtp_write_u16(buf, pos, status));
			}
		} else {
			/* Status Vector Chunk containing 14 1-bits symbols
			 * with short delta */
			uint16_t status = build_rtcpfb_chunk_status(
				chunk.ack, chunk.pos, false);
			CHECK(rtp_write_u16(buf, pos, status));
		}
	}

	for (i = 0; i < rtpfb->status_count; ++i) {
		int16_t delta = rtpfb->feedbacks[i].recv_delta;
		if (rtpfb->feedbacks[i].pkt_status_symbol == 1)
			CHECK(rtp_write_u8(buf, pos, delta));
		else if (rtpfb->feedbacks[i].pkt_status_symbol == 2)
			CHECK(rtp_write_u16(buf, pos, delta));
	}

	while ((*pos) % 4)
		CHECK(rtp_write_u8(buf, pos, 0));

	/* Write header */
	memset(&header, 0, sizeof(header));
	header.flags =
		(RTCP_PKT_VERSION << RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT) |
		(15 << RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT);
	header.type = RTCP_PKT_TYPE_RTPFB;
	header.len = ((*pos - header_pos) / 4) - 1;
	CHECK(rtcp_pkt_write_header(buf, &header_pos, &header));

out:
	return res;
}


static int rtcp_pkt_read_header(const struct pomp_buffer *buf,
				size_t *pos,
				struct rtcp_pkt_header *header)
{
	int res = 0;
	CHECK(rtp_read_u8(buf, pos, &header->flags));
	CHECK(rtp_read_u8(buf, pos, &header->type));
	CHECK(rtp_read_u16(buf, pos, &header->len));
out:
	return res;
}


/**
 * 6.4 Sender and Receiver Reports
 * 6.4.1 SR: Sender Report RTCP Packet
 * 6.4.2 RR: Receiver Report RTCP Packet
 */
static int rtcp_pkt_read_report_block(const struct pomp_buffer *buf,
				      size_t *pos,
				      const struct rtcp_pkt_header *header,
				      struct rtcp_pkt_report_block *rb)
{
	int res = 0;
	uint32_t fraction_lost = 0;
	CHECK(rtp_read_u32(buf, pos, &rb->ssrc));
	CHECK(rtp_read_u32(buf, pos, &fraction_lost));
	rb->fraction = (fraction_lost >> 24) & 0xff;
	rb->lost = fraction_lost & 0xffffff;
	CHECK(rtp_read_u32(buf, pos, &rb->ext_highest_seqnum));
	CHECK(rtp_read_u32(buf, pos, &rb->jitter));
	CHECK(rtp_read_u16(buf, pos, &rb->lsr.seconds));
	CHECK(rtp_read_u16(buf, pos, &rb->lsr.fraction));
	CHECK(rtp_read_u32(buf, pos, &rb->dlsr));
out:
	return res;
}


/**
 * 6.4.1 SR: Sender Report RTCP Packet
 */
static int rtcp_pkt_read_sender_report(const struct pomp_buffer *buf,
				       size_t *pos,
				       size_t end,
				       const struct rtcp_pkt_header *header,
				       const struct rtcp_pkt_read_cbs *cbs,
				       void *userdata)
{
	int res = 0;
	struct rtcp_pkt_sender_report sr;
	memset(&sr, 0, sizeof(sr));

	CHECK(rtp_read_u32(buf, pos, &sr.ssrc));
	CHECK(rtp_read_u32(buf, pos, &sr.ntp_timestamp.seconds));
	CHECK(rtp_read_u32(buf, pos, &sr.ntp_timestamp.fraction));
	CHECK(rtp_read_u32(buf, pos, &sr.rtp_timestamp));
	CHECK(rtp_read_u32(buf, pos, &sr.sender_packet_count));
	CHECK(rtp_read_u32(buf, pos, &sr.sender_byte_count));

	sr.report_count = RTCP_PKT_HEADER_FLAGS(COUNT, header->flags);
	for (uint32_t i = 0; i < sr.report_count; i++) {
		CHECK(rtcp_pkt_read_report_block(
			buf, pos, header, &sr.reports[i]));
	}

	if (cbs->sender_report != NULL)
		(*cbs->sender_report)(&sr, userdata);

out:
	return res;
}


/**
 * 6.4.2 RR: Receiver Report RTCP Packet
 */
static int rtcp_pkt_read_receiver_report(const struct pomp_buffer *buf,
					 size_t *pos,
					 size_t end,
					 const struct rtcp_pkt_header *header,
					 const struct rtcp_pkt_read_cbs *cbs,
					 void *userdata)
{
	int res = 0;
	struct rtcp_pkt_receiver_report rr;
	memset(&rr, 0, sizeof(rr));

	CHECK(rtp_read_u32(buf, pos, &rr.ssrc));

	rr.report_count = RTCP_PKT_HEADER_FLAGS(COUNT, header->flags);
	for (uint32_t i = 0; i < rr.report_count; i++) {
		CHECK(rtcp_pkt_read_report_block(
			buf, pos, header, &rr.reports[i]));
	}

	if (cbs->receiver_report != NULL)
		(*cbs->receiver_report)(&rr, userdata);

out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 * 6.5.8 PRIV: Private Extensions SDES Item
 */
static int rtcp_pkt_read_sdes_item(const struct pomp_buffer *buf,
				   size_t *pos,
				   size_t end,
				   const struct rtcp_pkt_header *header,
				   uint32_t ssrc,
				   const struct rtcp_pkt_read_cbs *cbs,
				   void *userdata)
{
	int res = 0;
	struct rtcp_pkt_sdes_item item;
	memset(&item, 0, sizeof(item));

	CHECK(rtp_read_u8(buf, pos, &item.type));

	CHECK(rtp_read_u8(buf, pos, &item.data_len));
	if (item.data_len > end - *pos) {
		res = -EIO;
		ULOGE("sdes: bad length: %zu (%u)", end - *pos, item.data_len);
		goto out;
	}

	if (item.data_len != 0) {
		CHECK(pomp_buffer_cread(
			buf, pos, (const void **)&item.data, item.data_len));
		if (item.type == RTCP_PKT_SDES_TYPE_PRIV) {
			/* Private item data: prefix_len + prefix + value */
			item.priv.prefix_len = *item.data;
			if (item.priv.prefix_len + 1 > item.data_len) {
				res = -EIO;
				ULOGE("sdes: bad prefix length: %u (%u)",
				      item.priv.prefix_len,
				      item.data_len);
				goto out;
			}

			/* Setup private item fields */
			item.priv.prefix = item.data + 1;
			item.priv.value_len =
				item.data_len - item.priv.prefix_len - 1;
			item.priv.value = item.data + item.priv.prefix_len + 1;
		}
	}

	if (cbs->sdes_item != NULL)
		(*cbs->sdes_item)(ssrc, &item, userdata);

out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 */
static int rtcp_pkt_read_sdes_chunk(const struct pomp_buffer *buf,
				    size_t *pos,
				    size_t end,
				    const struct rtcp_pkt_header *header,
				    const struct rtcp_pkt_read_cbs *cbs,
				    void *userdata)
{
	int res = 0;
	uint8_t type = 0;
	uint32_t ssrc = 0;

	CHECK(rtp_read_u32(buf, pos, &ssrc));

	while (*pos < end) {
		/* Read item type */
		CHECK(rtp_read_u8(buf, pos, &type));
		if (type == RTCP_PKT_SDES_TYPE_END)
			break;

		/* Rewind and read item */
		*pos -= 1;
		CHECK(rtcp_pkt_read_sdes_item(
			buf, pos, end, header, ssrc, cbs, userdata));
	}

	/* Align */
	while (*pos < end && *pos % 4 != 0)
		*pos += 1;

out:
	return res;
}


/**
 * 6.5 SDES: Source Description RTCP Packet
 */
static int rtcp_pkt_read_sdes(const struct pomp_buffer *buf,
			      size_t *pos,
			      size_t end,
			      const struct rtcp_pkt_header *header,
			      const struct rtcp_pkt_read_cbs *cbs,
			      void *userdata)
{
	int res = 0;
	uint32_t chunk_count = 0;

	chunk_count = RTCP_PKT_HEADER_FLAGS(COUNT, header->flags);
	for (uint32_t i = 0; i < chunk_count; i++)
		CHECK(rtcp_pkt_read_sdes_chunk(
			buf, pos, end, header, cbs, userdata));

out:
	return res;
}


/**
 * 6.6 BYE: Goodbye RTCP Packet
 */
static int rtcp_pkt_read_bye(const struct pomp_buffer *buf,
			     size_t *pos,
			     size_t end,
			     const struct rtcp_pkt_header *header,
			     const struct rtcp_pkt_read_cbs *cbs,
			     void *userdata)
{
	int res = 0;
	struct rtcp_pkt_bye bye;
	memset(&bye, 0, sizeof(bye));

	/* Read sources */
	bye.source_count = RTCP_PKT_HEADER_FLAGS(COUNT, header->flags);
	for (uint32_t i = 0; i < bye.source_count; i++)
		CHECK(rtp_read_u32(buf, pos, &bye.sources[i]));

	/* Read optional reason */
	if (*pos < end) {
		CHECK(rtp_read_u8(buf, pos, &bye.reason_len));
		if (end - *pos < bye.reason_len) {
			res = -EIO;
			ULOGW("bye: bad length: %zu (%u)",
			      end - *pos,
			      bye.reason_len);
			goto out;
		}
		CHECK(pomp_buffer_cread(
			buf, pos, (const void **)&bye.reason, bye.reason_len));
	}

	if (cbs->bye != NULL)
		(*cbs->bye)(&bye, userdata);

out:
	return res;
}


/**
 * 6.7 APP: Application-Defined RTCP Packet
 */
static int rtcp_pkt_read_app(const struct pomp_buffer *buf,
			     size_t *pos,
			     size_t end,
			     const struct rtcp_pkt_header *header,
			     const struct rtcp_pkt_read_cbs *cbs,
			     void *userdata)
{
	int res = 0;
	struct rtcp_pkt_app app;
	memset(&app, 0, sizeof(app));

	/* Read data */
	app.subtype = RTCP_PKT_HEADER_FLAGS(COUNT, header->flags);
	CHECK(rtp_read_u32(buf, pos, &app.ssrc));
	CHECK(rtp_read_u32(buf, pos, &app.name));
	if (*pos < end) {
		app.data_len = end - *pos;
		CHECK(pomp_buffer_cread(
			buf, pos, (const void **)&app.data, app.data_len));
	}

	if (cbs->app != NULL)
		(*cbs->app)(&app, userdata);

out:
	return res;
}


static int
rtcp_pkt_read_rtpfb_run_length_chunk(uint16_t chunk,
				     const struct rtcp_pkt_rtpfb_report *report,
				     size_t *pos)
{
	int res = 0;
	size_t j = *pos;
	size_t len = (chunk & 0x1FFF) + j;
	uint8_t status_symbol;

	CHECK(report->status_count - len);
	status_symbol = (chunk >> RUN_LENGTH_CHUNK_ACK_LG) & 0x03;
	for (; j < len; j++) {
		report->feedbacks[j].seq_num = report->base_seq + j;
		report->feedbacks[j].pkt_status_symbol = status_symbol;
	}
	*pos = j;

out:
	return res;
}


static int rtcp_pkt_read_rtpfb_status_vector_chunk(
	uint16_t chunk,
	const struct rtcp_pkt_rtpfb_report *report,
	size_t *pos)
{
	int res = 0;
	size_t len = 0;
	size_t i = *pos;
	size_t j = 1;

	if ((chunk & 0x4000) == 0) {
		/* Small delta chunk */
		len = STATUS_VECTOR_CHUNK_ACK_LG;
		if (len + i >= report->status_count)
			len = report->status_count - i;
		CHECK(report->status_count - len);
		while (j <= len) {
			const uint8_t offset = STATUS_VECTOR_CHUNK_ACK_LG - j;
			const uint8_t symbol = (chunk >> offset) & 0x01;
			report->feedbacks[i].pkt_status_symbol = symbol;
			report->feedbacks[i].seq_num = report->base_seq + i;
			++i;
			++j;
		}
	} else {
		/* Large delta chunk */
		len = STATUS_VECTOR_CHUNK_ACK_LG / 2;
		if (len + i >= report->status_count)
			len = report->status_count - i;
		CHECK(report->status_count - len);
		while (j <= len) {
			const uint8_t offset =
				STATUS_VECTOR_CHUNK_ACK_LG - j * 2;
			const uint8_t symbol = (chunk >> offset) & 0x03;
			report->feedbacks[i].pkt_status_symbol = symbol;
			report->feedbacks[i].seq_num = report->base_seq + i;
			++i;
			++j;
		}
	}
	*pos = i;

out:
	return res;
}


static int rtcp_pkt_read_rtpfb(const struct pomp_buffer *buf,
			       size_t *pos,
			       size_t end,
			       const struct rtcp_pkt_header *header,
			       const struct rtcp_pkt_read_cbs *cbs,
			       void *userdata)
{
	int res = 0;
	size_t i = 0;
	struct rtcp_pkt_rtpfb_report report;
	report.status_count = 0;
	report.feedbacks = NULL;
	CHECK(rtp_read_u32(buf, pos, &report.sender_ssrc));
	CHECK(rtp_read_u32(buf, pos, &report.media_ssrc));
	CHECK(rtp_read_u16(buf, pos, &report.base_seq));
	CHECK(rtp_read_u16(buf, pos, &report.status_count));
	CHECK(rtp_read_u32(buf, pos, &report.ref_time));
	report.feedback_pkt_count = report.ref_time & 0xFF;
	report.ref_time = report.ref_time >> 8;

	if (report.status_count > RTPFB_MAX_PKT) {
		res = -E2BIG;
		goto out;
	}
	report.feedbacks = calloc(report.status_count,
				  sizeof(struct rtcp_pkt_rtpfb_feedback));
	if (report.feedbacks == NULL) {
		res = -ENOMEM;
		goto out;
	}

	while (i < report.status_count) {
		uint16_t chunk;
		CHECK(rtp_read_u16(buf, pos, &chunk));
		if ((chunk & 0x8000) == 0) {
			rtcp_pkt_read_rtpfb_run_length_chunk(
				chunk, &report, &i);
		} else {
			rtcp_pkt_read_rtpfb_status_vector_chunk(
				chunk, &report, &i);
		}
	}

	uint8_t short_delta;
	uint16_t long_delta;
	for (i = 0; i < report.status_count; ++i) {
		switch (report.feedbacks[i].pkt_status_symbol) {
		case 0:
		case 3:
			report.feedbacks[i].recv_delta = 0;
			break;
		case 1:
			CHECK(rtp_read_u8(buf, pos, &short_delta));
			report.feedbacks[i].recv_delta = short_delta;
			break;
		case 2:
			CHECK(rtp_read_u16(buf, pos, &long_delta));
			report.feedbacks[i].recv_delta = long_delta;
			break;
		}
	}

	if (cbs->rtpfb_report != NULL)
		(*cbs->rtpfb_report)(&report, userdata);

out:
	if (report.feedbacks != NULL)
		free(report.feedbacks);

	return res;
}


int rtcp_pkt_read(const struct pomp_buffer *buf,
		  const struct rtcp_pkt_read_cbs *cbs,
		  void *userdata)
{
	int res = 0;
	struct rtcp_pkt_header header;
	uint8_t version = 0;
	size_t len = 0, end = 0, pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);

	pomp_buffer_get_cdata(buf, NULL, &len, NULL);
	while (pos < len) {
		/* Read header */
		if (len - pos < RTCP_PKT_HEADER_SIZE) {
			res = -EIO;
			ULOGE("hdr: bad length: %zu (%u)",
			      len - pos,
			      RTCP_PKT_HEADER_SIZE);
			goto out;
		}
		memset(&header, 0, sizeof(header));
		CHECK(rtcp_pkt_read_header(buf, &pos, &header));

		/* Check version */
		version = RTCP_PKT_HEADER_FLAGS(VERSION, header.flags);
		if (version != RTCP_PKT_VERSION) {
			res = -EIO;
			ULOGE("hdr: bad version: %u (%u)",
			      version,
			      RTCP_PKT_VERSION);
			goto out;
		}

		/* Check length; the length in the header is the number
		 * of 32-bit words in the packet, not including the
		 * header itself */
		if (len - pos < header.len * 4) {
			res = -EIO;
			ULOGE("hdr: bad length: %zu (%u)",
			      len - pos,
			      header.len * 4);
			goto out;
		}

		/* Ignore errors during payload to try to read other packets */
		end = pos + header.len * 4;
		switch (header.type) {
		case RTCP_PKT_TYPE_SR:
			rtcp_pkt_read_sender_report(
				buf, &pos, end, &header, cbs, userdata);
			break;
		case RTCP_PKT_TYPE_RR:
			rtcp_pkt_read_receiver_report(
				buf, &pos, end, &header, cbs, userdata);
			break;
		case RTCP_PKT_TYPE_SDES:
			rtcp_pkt_read_sdes(
				buf, &pos, end, &header, cbs, userdata);
			break;
		case RTCP_PKT_TYPE_BYE:
			rtcp_pkt_read_bye(
				buf, &pos, end, &header, cbs, userdata);
			break;
		case RTCP_PKT_TYPE_APP:
			rtcp_pkt_read_app(
				buf, &pos, end, &header, cbs, userdata);
			break;
		case RTCP_PKT_TYPE_RTPFB:
			rtcp_pkt_read_rtpfb(
				buf, &pos, end, &header, cbs, userdata);
		}

		/* In any case, continue after the payload based on the length
		 * given in the header */
		pos = end;
	}

out:
	return res;
}
