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

#include "rtp_test.h"

#include "rtp/rtp_pkt.h"


/* Minimal 12-byte header: V=2 P=0 X=0 CC=0 M=0 PT=96 seqnum=0x1234
 * timestamp=0x89ABCDEF ssrc=0x12345678 */
static const uint8_t k_min_hdr[] = {
	0x80,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
};

/* Same header + 4 bytes of payload */
static const uint8_t k_with_payload[] = {
	0x80,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
	0x01,
	0x02,
	0x03,
	0x04,
};

/* CC=2 in flags, but only 1 CSRC's worth of extra bytes -> overrun */
static const uint8_t k_csrc_overrun[] = {
	0x82,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
	0xAA,
	0xAA,
	0xAA,
	0xAA,
};

/* CC=2, 2 full CSRC entries + 4 bytes payload */
static const uint8_t k_with_csrc[] = {
	0x82, 0x60, 0x12, 0x34, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78,
	0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB, 0x01, 0x02, 0x03, 0x04,
};

/* X=1, only 2 of the 4 extension-header bytes present */
static const uint8_t k_ext_header_overrun[] = {
	0x90,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
	0x12,
	0x34,
};

/* X=1, ext header declares len=1 word (4 bytes) but only 2 bytes follow */
static const uint8_t k_ext_data_overrun[] = {
	0x90,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
	0x12,
	0x34,
	0x00,
	0x01,
	0xDE,
	0xAD,
};

/* X=1, CC=2: full header + 2 CSRC + ext header (id=0x1234, len=1 word) +
 * 4 bytes ext data + 4 bytes payload */
static const uint8_t k_with_extension[] = {
	0x92, 0x60, 0x12, 0x34, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56,
	0x78, 0xAA, 0xAA, 0xAA, 0xAA, 0xBB, 0xBB, 0xBB, 0xBB, 0x12, 0x34,
	0x00, 0x01, 0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04,
};

/* P=1, no payload/padding bytes at all -> no room for the pad-length byte */
static const uint8_t k_padding_no_room[] = {
	0xA0,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
};

/* P=1, 4-byte payload area, last byte claims 5 bytes of padding (too many) */
static const uint8_t k_padding_too_large[] = {
	0xA0,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
	0x01,
	0x02,
	0x03,
	0x05,
};

/* P=1, 8-byte payload area, last byte declares 4 bytes of padding */
static const uint8_t k_with_padding[] = {
	0xA0, 0x60, 0x12, 0x34, 0x89, 0xAB, 0xCD, 0xEF, 0x12, 0x34,
	0x56, 0x78, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00, 0x04,
};

/* Buffer too short for even the fixed 12-byte header */
static const uint8_t k_too_short[] = {
	0x80,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
};

/* Version bits set to 1 instead of 2 */
static const uint8_t k_bad_version[] = {
	0x40,
	0x60,
	0x12,
	0x34,
	0x89,
	0xAB,
	0xCD,
	0xEF,
	0x12,
	0x34,
	0x56,
	0x78,
};


static void test_rtp_pkt_new_ok(void)
{
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pkt);
	CU_ASSERT_EQUAL(pkt->header.flags, 0);
	CU_ASSERT_PTR_NULL(pkt->raw.buf);

	CU_ASSERT_EQUAL(rtp_pkt_destroy(pkt), 0);
}


static void test_rtp_pkt_new_einval(void)
{
	CU_ASSERT_EQUAL(rtp_pkt_new(NULL), -EINVAL);
}


static void test_rtp_pkt_destroy_null_is_noop(void)
{
	CU_ASSERT_EQUAL(rtp_pkt_destroy(NULL), 0);
}


static void test_rtp_pkt_clone_einval_null_ret_obj(void)
{
	struct rtp_pkt pkt;
	memset(&pkt, 0, sizeof(pkt));

	CU_ASSERT_EQUAL(rtp_pkt_clone(&pkt, NULL), -EINVAL);
}


static void test_rtp_pkt_clone_ok_no_buf(void)
{
	struct rtp_pkt *pkt = NULL;
	struct rtp_pkt *clone = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	fill_rtp_pkt_header(&pkt->header,
			    2,
			    0,
			    0,
			    0,
			    0,
			    96,
			    0x1234,
			    0x89ABCDEF,
			    0x12345678);

	CU_ASSERT_EQUAL(rtp_pkt_clone(pkt, &clone), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(clone);
	compare_rtp_pkt_header(&pkt->header, &clone->header);
	CU_ASSERT_PTR_NULL(clone->raw.buf);

	rtp_pkt_destroy(pkt);
	rtp_pkt_destroy(clone);
}


static void test_rtp_pkt_clone_ok_with_buf_shares_ref(void)
{
	struct rtp_pkt *pkt = NULL;
	struct rtp_pkt *clone = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	pkt->raw.buf =
		make_pomp_buffer_from_bytes(k_min_hdr, sizeof(k_min_hdr));

	CU_ASSERT_EQUAL(rtp_pkt_clone(pkt, &clone), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(clone);
	CU_ASSERT_PTR_EQUAL(clone->raw.buf, pkt->raw.buf);

	/* Destroy original first, then clone: must not crash/double-free */
	rtp_pkt_destroy(pkt);
	rtp_pkt_destroy(clone);
}


static void test_rtp_pkt_clone_ok_with_buf_destroy_reverse_order(void)
{
	struct rtp_pkt *pkt = NULL;
	struct rtp_pkt *clone = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	pkt->raw.buf =
		make_pomp_buffer_from_bytes(k_min_hdr, sizeof(k_min_hdr));

	CU_ASSERT_EQUAL(rtp_pkt_clone(pkt, &clone), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(clone);

	/* Destroy clone first, then original: must not crash/double-free */
	rtp_pkt_destroy(clone);
	rtp_pkt_destroy(pkt);
}


static void test_rtp_pkt_finalize_header_einval_null_pkt(void)
{
	CU_ASSERT_EQUAL(rtp_pkt_finalize_header(NULL), -EINVAL);
}


static void test_rtp_pkt_finalize_header_einval_null_buf(void)
{
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_finalize_header(pkt), -EINVAL);

	rtp_pkt_destroy(pkt);
}


static void test_rtp_pkt_finalize_header_einval_short_buf(void)
{
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	pkt->raw.buf = make_pomp_buffer(8);
	pkt->raw.len = 8;

	CU_ASSERT_EQUAL(rtp_pkt_finalize_header(pkt), -EINVAL);

	rtp_pkt_destroy(pkt);
}


static void test_rtp_pkt_finalize_header_ok(void)
{
	struct rtp_pkt *pkt = NULL;
	const uint8_t *cdata = NULL;
	size_t len = 0;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	pkt->raw.buf = make_pomp_buffer(RTP_PKT_HEADER_SIZE);
	pkt->raw.len = RTP_PKT_HEADER_SIZE;
	fill_rtp_pkt_header(&pkt->header,
			    2,
			    0,
			    0,
			    0,
			    0,
			    96,
			    0x1234,
			    0x89ABCDEF,
			    0x12345678);

	CU_ASSERT_EQUAL(rtp_pkt_finalize_header(pkt), 0);

	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(
			pkt->raw.buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, sizeof(k_min_hdr));
	CU_ASSERT_EQUAL(memcmp(cdata, k_min_hdr, sizeof(k_min_hdr)), 0);

	rtp_pkt_destroy(pkt);
}


static void test_rtp_pkt_read_einval_null_buf(void)
{
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(NULL, pkt), -EINVAL);

	rtp_pkt_destroy(pkt);
}


static void test_rtp_pkt_read_einval_null_pkt(void)
{
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(k_min_hdr, sizeof(k_min_hdr));

	CU_ASSERT_EQUAL(rtp_pkt_read(buf, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_too_short(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(k_too_short, sizeof(k_too_short));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	/* A ref on buf is taken unconditionally, even on failure: pkt must
	 * still be destroyed to release it. */
	CU_ASSERT_PTR_EQUAL(pkt->raw.buf, buf);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_bad_version(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_bad_version, sizeof(k_bad_version));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_ok_minimal(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(k_min_hdr, sizeof(k_min_hdr));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), 0);

	CU_ASSERT_EQUAL(pkt->header.flags, 0x8060);
	CU_ASSERT_EQUAL(pkt->header.seqnum, 0x1234);
	CU_ASSERT_EQUAL(pkt->header.timestamp, 0x89ABCDEF);
	CU_ASSERT_EQUAL(pkt->header.ssrc, 0x12345678);
	CU_ASSERT_EQUAL(pkt->payload.off, RTP_PKT_HEADER_SIZE);
	CU_ASSERT_EQUAL(pkt->payload.len, 0);
	CU_ASSERT_EQUAL(pkt->padding.len, 0);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_ok_with_payload(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_with_payload, sizeof(k_with_payload));
	const uint8_t *cdata = NULL;
	size_t len = 0;

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), 0);

	CU_ASSERT_EQUAL(pkt->payload.off, 12);
	CU_ASSERT_EQUAL(pkt->payload.len, 4);

	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(memcmp(cdata + pkt->payload.off, "\x01\x02\x03\x04", 4),
			0);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_csrc_overrun(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_csrc_overrun, sizeof(k_csrc_overrun));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_ok_with_csrc(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(k_with_csrc, sizeof(k_with_csrc));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), 0);

	CU_ASSERT_EQUAL(pkt->payload.off, 20);
	CU_ASSERT_EQUAL(pkt->payload.len, 4);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_ext_header_overrun(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_ext_header_overrun, sizeof(k_ext_header_overrun));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_ext_data_overrun(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_ext_data_overrun, sizeof(k_ext_data_overrun));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_ok_with_extension(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_with_extension, sizeof(k_with_extension));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), 0);

	CU_ASSERT_EQUAL(pkt->extheader.id, 0x1234);
	CU_ASSERT_EQUAL(pkt->extheader.off, 20);
	CU_ASSERT_EQUAL(pkt->extheader.len, 8);
	CU_ASSERT_EQUAL(pkt->payload.off, 28);
	CU_ASSERT_EQUAL(pkt->payload.len, 4);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_padding_no_room(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_padding_no_room, sizeof(k_padding_no_room));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_eio_padding_too_large(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_padding_too_large, sizeof(k_padding_too_large));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_ok_with_padding(void)
{
	struct rtp_pkt *pkt = NULL;
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
		k_with_padding, sizeof(k_with_padding));

	CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
	CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), 0);

	CU_ASSERT_EQUAL(pkt->payload.len, 4);
	CU_ASSERT_EQUAL(pkt->padding.off, 16);
	CU_ASSERT_EQUAL(pkt->padding.len, 4);

	rtp_pkt_destroy(pkt);
	pomp_buffer_unref(buf);
}


static void test_rtp_pkt_read_must_destroy_even_on_error(void)
{
	static const struct {
		const uint8_t *data;
		size_t len;
	} invalid[] = {
		{k_too_short, sizeof(k_too_short)},
		{k_bad_version, sizeof(k_bad_version)},
		{k_csrc_overrun, sizeof(k_csrc_overrun)},
		{k_ext_header_overrun, sizeof(k_ext_header_overrun)},
		{k_ext_data_overrun, sizeof(k_ext_data_overrun)},
		{k_padding_no_room, sizeof(k_padding_no_room)},
		{k_padding_too_large, sizeof(k_padding_too_large)},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(invalid); i++) {
		struct rtp_pkt *pkt = NULL;
		struct pomp_buffer *buf = make_pomp_buffer_from_bytes(
			invalid[i].data, invalid[i].len);

		CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
		CU_ASSERT_EQUAL(rtp_pkt_read(buf, pkt), -EIO);
		CU_ASSERT_EQUAL(rtp_pkt_destroy(pkt), 0);

		pomp_buffer_unref(buf);
	}
}


CU_TestInfo s_test_rtp_pkt[] = {
	{FN("new_ok"), &test_rtp_pkt_new_ok},
	{FN("new_einval"), &test_rtp_pkt_new_einval},
	{FN("destroy_null_is_noop"), &test_rtp_pkt_destroy_null_is_noop},
	{FN("clone_einval_null_ret_obj"),
	 &test_rtp_pkt_clone_einval_null_ret_obj},
	{FN("clone_ok_no_buf"), &test_rtp_pkt_clone_ok_no_buf},
	{FN("clone_ok_with_buf_shares_ref"),
	 &test_rtp_pkt_clone_ok_with_buf_shares_ref},
	{FN("clone_ok_with_buf_destroy_reverse_order"),
	 &test_rtp_pkt_clone_ok_with_buf_destroy_reverse_order},
	{FN("finalize_header_einval_null_pkt"),
	 &test_rtp_pkt_finalize_header_einval_null_pkt},
	{FN("finalize_header_einval_null_buf"),
	 &test_rtp_pkt_finalize_header_einval_null_buf},
	{FN("finalize_header_einval_short_buf"),
	 &test_rtp_pkt_finalize_header_einval_short_buf},
	{FN("finalize_header_ok"), &test_rtp_pkt_finalize_header_ok},
	{FN("read_einval_null_buf"), &test_rtp_pkt_read_einval_null_buf},
	{FN("read_einval_null_pkt"), &test_rtp_pkt_read_einval_null_pkt},
	{FN("read_eio_too_short"), &test_rtp_pkt_read_eio_too_short},
	{FN("read_eio_bad_version"), &test_rtp_pkt_read_eio_bad_version},
	{FN("read_ok_minimal"), &test_rtp_pkt_read_ok_minimal},
	{FN("read_ok_with_payload"), &test_rtp_pkt_read_ok_with_payload},
	{FN("read_eio_csrc_overrun"), &test_rtp_pkt_read_eio_csrc_overrun},
	{FN("read_ok_with_csrc"), &test_rtp_pkt_read_ok_with_csrc},
	{FN("read_eio_ext_header_overrun"),
	 &test_rtp_pkt_read_eio_ext_header_overrun},
	{FN("read_eio_ext_data_overrun"),
	 &test_rtp_pkt_read_eio_ext_data_overrun},
	{FN("read_ok_with_extension"), &test_rtp_pkt_read_ok_with_extension},
	{FN("read_eio_padding_no_room"),
	 &test_rtp_pkt_read_eio_padding_no_room},
	{FN("read_eio_padding_too_large"),
	 &test_rtp_pkt_read_eio_padding_too_large},
	{FN("read_ok_with_padding"), &test_rtp_pkt_read_ok_with_padding},
	{FN("read_must_destroy_even_on_error"),
	 &test_rtp_pkt_read_must_destroy_even_on_error},
	CU_TEST_INFO_NULL,
};
