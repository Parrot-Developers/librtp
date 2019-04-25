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

/* clang-format off */
/* codecheck_ignore[COMPLEX_MACRO] */
#define CHECK(_x) if ((res = (_x)) < 0) goto out
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


static int rtcp_pkt_read_header(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_report_block(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_sender_report(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_receiver_report(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_sdes_item(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_sdes_chunk(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_sdes(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_bye(struct pomp_buffer *buf,
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
static int rtcp_pkt_read_app(struct pomp_buffer *buf,
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


int rtcp_pkt_read(struct pomp_buffer *buf,
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
		}

		/* In any case, continue after the payload based on the length
		 * given in the header */
		pos = end;
	}

out:
	return res;
}
