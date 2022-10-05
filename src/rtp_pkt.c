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

#include "rtp_priv.h"

ULOG_DECLARE_TAG(rtp);

/* clang-format off */
#define CHECK(_x) do { if ((res = (_x)) < 0) goto out; } while (0)
/* clang-format on */

int rtp_pkt_new(struct rtp_pkt **ret_obj)
{
	struct rtp_pkt *pkt = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	pkt = calloc(1, sizeof(*pkt));
	if (pkt == NULL)
		return -ENOMEM;
	list_node_unref(&pkt->node);

	*ret_obj = pkt;
	return 0;
}


int rtp_pkt_clone(const struct rtp_pkt *pkt, struct rtp_pkt **ret_obj)
{
	struct rtp_pkt *new_pkt = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	new_pkt = calloc(1, sizeof(*new_pkt));
	if (new_pkt == NULL)
		return -ENOMEM;

	/* Copy contents, and add a ref on internal buffer */
	*new_pkt = *pkt;
	list_node_unref(&new_pkt->node);
	if (pkt->raw.buf != NULL)
		pomp_buffer_ref(pkt->raw.buf);

	*ret_obj = new_pkt;
	return 0;
}


int rtp_pkt_destroy(struct rtp_pkt *pkt)
{
	if (pkt == NULL)
		return 0;

	if (list_node_is_ref(&pkt->node))
		ULOGW("packet %p is still in a list", pkt);
	if (pkt->raw.buf != NULL)
		pomp_buffer_unref(pkt->raw.buf);
	free(pkt);
	return 0;
}


static int rtp_pkt_write_header(struct pomp_buffer *buf,
				size_t *pos,
				const struct rtp_pkt_header *header)
{
	int res = 0;
	CHECK(rtp_write_u16(buf, pos, header->flags));
	CHECK(rtp_write_u16(buf, pos, header->seqnum));
	CHECK(rtp_write_u32(buf, pos, header->timestamp));
	CHECK(rtp_write_u32(buf, pos, header->ssrc));
out:
	return res;
}


int rtp_pkt_finalize_header(struct rtp_pkt *pkt)
{
	int res = 0;
	size_t pos = 0;

	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt->raw.buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt->raw.len < RTP_PKT_HEADER_SIZE, EINVAL);

	res = rtp_pkt_write_header(pkt->raw.buf, &pos, &pkt->header);

	return res;
}


static int rtp_pkt_read_header(const struct pomp_buffer *buf,
			       size_t *pos,
			       struct rtp_pkt_header *header)
{
	int res = 0;
	CHECK(rtp_read_u16(buf, pos, &header->flags));
	CHECK(rtp_read_u16(buf, pos, &header->seqnum));
	CHECK(rtp_read_u32(buf, pos, &header->timestamp));
	CHECK(rtp_read_u32(buf, pos, &header->ssrc));
out:
	return res;
}


int rtp_pkt_read(struct pomp_buffer *buf, struct rtp_pkt *pkt)
{
	int res = 0;
	size_t pos = 0;
	uint32_t version = 0, csrc_count = 0;
	uint16_t u16 = 0;
	uint8_t padding = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	/* Get start/end of buffer */
	pomp_buffer_ref(buf);
	pkt->raw.buf = buf;
	pomp_buffer_get_cdata(
		buf, (const void **)&pkt->raw.cdata, &pkt->raw.len, NULL);

	/* Read header */
	if (pkt->raw.len < RTP_PKT_HEADER_SIZE) {
		res = -EIO;
		ULOGE("rtp: bad length: %zu (%u)",
		      pkt->raw.len,
		      RTP_PKT_HEADER_SIZE);
		goto out;
	}
	CHECK(rtp_pkt_read_header(buf, &pos, &pkt->header));

	/* Check version */
	version = RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, VERSION);
	if (version != RTP_PKT_VERSION) {
		res = -EIO;
		ULOGE("rtp: bad version: %u (%u)", version, RTP_PKT_VERSION);
		goto out;
	}

	/* Skip CSRC */
	csrc_count = RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, CSRC);
	if (csrc_count > 0) {
		if (pkt->raw.len - pos < csrc_count * 4) {
			res = -EIO;
			ULOGE("rtp: bad length: %zu (%u)",
			      pkt->raw.len - pos,
			      csrc_count * 4);
			goto out;
		}
		pos += csrc_count * 4;
	}

	/* Do we have an extension header? */
	if (RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, EXTENSION)) {
		if (pkt->raw.len - pos < 4) {
			res = -EIO;
			ULOGE("rtp: bad length: %zu (%u)",
			      pkt->raw.len - pos,
			      4);
			goto out;
		}
		pkt->extheader.off = pos;

		/* Read extension id and length (number of 32-bit not
		 * counting type + length itself) */
		CHECK(rtp_read_u16(buf, &pos, &pkt->extheader.id));
		CHECK(rtp_read_u16(buf, &pos, &u16));
		pkt->extheader.len = u16 * 4 + 4;

		if (pkt->raw.len - pos < (size_t)u16 * 4) {
			res = -EIO;
			ULOGE("rtp: bad length: %zu (%u)",
			      pkt->raw.len - pos,
			      u16 * 4);
			goto out;
		}
		pos += u16 * 4;
	}

	/* Setup payload */
	pkt->payload.off = pos;
	pkt->payload.len = pkt->raw.len - pos;

	/* Are there padding bytes? */
	if (RTP_PKT_HEADER_FLAGS_GET(pkt->header.flags, PADDING)) {
		if (pkt->payload.len < 1) {
			res = -EIO;
			ULOGE("rtp: bad length: %zu (%u)", pkt->payload.len, 1);
			goto out;
		}
		pos = pkt->raw.len - 1;
		CHECK(rtp_read_u8(buf, &pos, &padding));
		if (pkt->payload.len < padding) {
			res = -EIO;
			ULOGE("rtp: bad length: %zu (%u)",
			      pkt->payload.len,
			      padding);
			goto out;
		}
		pkt->payload.len -= padding;
		pkt->padding.off = pkt->payload.off + pkt->payload.len;
		pkt->padding.len = padding;
	}

out:
	return res;
}
