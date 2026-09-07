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

#include "rtp/rtcp_pkt.h"


#define MAX_CAPTURE_SDES_ITEMS 8
#define MAX_CAPTURE_RTPFB 64


struct sdes_item_capture {
	uint32_t ssrc;
	struct rtcp_pkt_sdes_item item;
};


struct rtcp_capture {
	struct rtcp_pkt_sender_report sr;
	bool sr_called;

	struct rtcp_pkt_receiver_report rr;
	bool rr_called;

	struct sdes_item_capture sdes_items[MAX_CAPTURE_SDES_ITEMS];
	int sdes_item_count;

	struct rtcp_pkt_bye bye;
	bool bye_called;

	struct rtcp_pkt_app app;
	bool app_called;

	struct rtcp_pkt_rtpfb_report rtpfb;
	struct rtcp_pkt_rtpfb_feedback rtpfb_feedbacks_copy[MAX_CAPTURE_RTPFB];
	bool rtpfb_called;
};


static void capture_reset(struct rtcp_capture *cap)
{
	memset(cap, 0, sizeof(*cap));
}


static void cb_sender_report(const struct rtcp_pkt_sender_report *sr,
			     void *userdata)
{
	struct rtcp_capture *cap = userdata;
	cap->sr = *sr;
	cap->sr_called = true;
}


static void cb_receiver_report(const struct rtcp_pkt_receiver_report *rr,
			       void *userdata)
{
	struct rtcp_capture *cap = userdata;
	cap->rr = *rr;
	cap->rr_called = true;
}


static void cb_sdes_item(uint32_t ssrc,
			 const struct rtcp_pkt_sdes_item *item,
			 void *userdata)
{
	struct rtcp_capture *cap = userdata;
	if (cap->sdes_item_count < MAX_CAPTURE_SDES_ITEMS) {
		cap->sdes_items[cap->sdes_item_count].ssrc = ssrc;
		cap->sdes_items[cap->sdes_item_count].item = *item;
		cap->sdes_item_count++;
	}
}


static void cb_bye(const struct rtcp_pkt_bye *bye, void *userdata)
{
	struct rtcp_capture *cap = userdata;
	cap->bye = *bye;
	cap->bye_called = true;
}


static void cb_app(const struct rtcp_pkt_app *app, void *userdata)
{
	struct rtcp_capture *cap = userdata;
	cap->app = *app;
	cap->app_called = true;
}


static void cb_rtpfb(const struct rtcp_pkt_rtpfb_report *rtpfb, void *userdata)
{
	struct rtcp_capture *cap = userdata;
	uint16_t n = rtpfb->status_count;
	if (n > MAX_CAPTURE_RTPFB)
		n = MAX_CAPTURE_RTPFB;

	cap->rtpfb = *rtpfb;
	if (n > 0) {
		memcpy(cap->rtpfb_feedbacks_copy,
		       rtpfb->feedbacks,
		       n * sizeof(rtpfb->feedbacks[0]));
	}
	/* Redirect to our own copy: the library frees rtpfb->feedbacks right
	 * after this callback returns. */
	cap->rtpfb.feedbacks = cap->rtpfb_feedbacks_copy;
	cap->rtpfb_called = true;
}


static const struct rtcp_pkt_read_cbs k_full_cbs = {
	.sender_report = cb_sender_report,
	.receiver_report = cb_receiver_report,
	.sdes_item = cb_sdes_item,
	.bye = cb_bye,
	.app = cb_app,
	.rtpfb_report = cb_rtpfb,
};


static void test_rtcp_pkt_type_str(void)
{
	static const struct {
		uint8_t type;
		const char *expected;
	} table[] = {
		/* NOTE: rtcp_pkt_type_str() has its SR/RR strings swapped in
		 * the current implementation (see src/rtcp_pkt.c) -- this
		 * documents the current, actual behavior verbatim, it is
		 * not a fix. */
		{RTCP_PKT_TYPE_SR, "RR"},
		{RTCP_PKT_TYPE_RR, "SR"},
		{RTCP_PKT_TYPE_SDES, "SDES"},
		{RTCP_PKT_TYPE_BYE, "BYE"},
		{RTCP_PKT_TYPE_APP, "APP"},
		{RTCP_PKT_TYPE_RTPFB, "RTPFB"},
		{0, "UNKNOWN"},
		{255, "UNKNOWN"},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_STRING_EQUAL(rtcp_pkt_type_str(table[i].type),
				       table[i].expected);
	}
}


static void test_rtcp_pkt_sdes_type_str(void)
{
	static const struct {
		uint8_t type;
		const char *expected;
	} table[] = {
		{RTCP_PKT_SDES_TYPE_END, "END"},
		{RTCP_PKT_SDES_TYPE_CNAME, "CNAME"},
		{RTCP_PKT_SDES_TYPE_NAME, "NAME"},
		{RTCP_PKT_SDES_TYPE_EMAIL, "EMAIL"},
		{RTCP_PKT_SDES_TYPE_PHONE, "PHONE"},
		{RTCP_PKT_SDES_TYPE_LOC, "LOC"},
		{RTCP_PKT_SDES_TYPE_TOOL, "TOOL"},
		{RTCP_PKT_SDES_TYPE_NOTE, "NOTE"},
		{RTCP_PKT_SDES_TYPE_PRIV, "PRIV"},
		{99, "UNKNOWN"},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_STRING_EQUAL(rtcp_pkt_sdes_type_str(table[i].type),
				       table[i].expected);
	}
}


static void test_rtcp_pkt_write_sender_report_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_sender_report sr;
	memset(&sr, 0, sizeof(sr));

	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(NULL, &pos, &sr), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(buf, NULL, &sr), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(buf, &pos, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_receiver_report_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_receiver_report rr;
	memset(&rr, 0, sizeof(rr));

	CU_ASSERT_EQUAL(rtcp_pkt_write_receiver_report(NULL, &pos, &rr),
			-EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_receiver_report(buf, NULL, &rr),
			-EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_receiver_report(buf, &pos, NULL),
			-EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_sdes_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_sdes sdes;
	memset(&sdes, 0, sizeof(sdes));

	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(NULL, &pos, &sdes), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, NULL, &sdes), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_bye_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_bye bye;
	memset(&bye, 0, sizeof(bye));

	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(NULL, &pos, &bye), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(buf, NULL, &bye), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(buf, &pos, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_app_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_app app;
	memset(&app, 0, sizeof(app));

	CU_ASSERT_EQUAL(rtcp_pkt_write_app(NULL, &pos, &app), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_app(buf, NULL, &app), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_app(buf, &pos, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_rtpfb_einval(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_rtpfb_report rtpfb;
	memset(&rtpfb, 0, sizeof(rtpfb));

	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(NULL, &pos, &rtpfb), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(buf, NULL, &rtpfb), -EINVAL);
	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(buf, &pos, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_write_sdes_priv_item_too_long(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	uint8_t prefix[200];
	uint8_t value[100];
	struct rtcp_pkt_sdes_item item;
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes sdes;

	memset(prefix, 'p', sizeof(prefix));
	memset(value, 'v', sizeof(value));
	memset(&item, 0, sizeof(item));
	item.type = RTCP_PKT_SDES_TYPE_PRIV;
	item.priv.prefix_len = sizeof(prefix);
	item.priv.prefix = prefix;
	item.priv.value_len = sizeof(value);
	item.priv.value = value;

	memset(&chunk, 0, sizeof(chunk));
	chunk.ssrc = 0x1;
	chunk.item_count = 1;
	chunk.items = &item;

	memset(&sdes, 0, sizeof(sdes));
	sdes.chunk_count = 1;
	sdes.chunks = &chunk;

	/* prefix_len + value_len + 1 = 301 > 255 */
	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, &sdes), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sender_report_no_reports(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_sender_report sr;
	struct rtcp_capture cap;

	memset(&sr, 0, sizeof(sr));
	sr.ssrc = 0x11223344;
	sr.ntp_timestamp.seconds = 100;
	sr.ntp_timestamp.fraction = 0x80000000;
	sr.rtp_timestamp = 0xAABBCCDD;
	sr.sender_packet_count = 42;
	sr.sender_byte_count = 12345;
	sr.report_count = 0;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(buf, &pos, &sr), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.sr_called);
	compare_rtcp_pkt_sender_report(&cap.sr, &sr);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sender_report_with_reports(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_sender_report sr;
	struct rtcp_capture cap;

	memset(&sr, 0, sizeof(sr));
	sr.ssrc = 0x11223344;
	sr.ntp_timestamp.seconds = 100;
	sr.ntp_timestamp.fraction = 0x80000000;
	sr.rtp_timestamp = 0xAABBCCDD;
	sr.sender_packet_count = 42;
	sr.sender_byte_count = 12345;
	sr.report_count = 2;

	sr.reports[0].ssrc = 0x1;
	sr.reports[0].fraction = 10;
	sr.reports[0].lost = -5;
	sr.reports[0].ext_highest_seqnum = 1000;
	sr.reports[0].jitter = 50;
	sr.reports[0].lsr.seconds = 1;
	sr.reports[0].lsr.fraction = 2;
	sr.reports[0].dlsr = 3;

	sr.reports[1].ssrc = 0x2;
	sr.reports[1].fraction = 20;
	sr.reports[1].lost = 1234;
	sr.reports[1].ext_highest_seqnum = 2000;
	sr.reports[1].jitter = 60;
	sr.reports[1].lsr.seconds = 4;
	sr.reports[1].lsr.fraction = 5;
	sr.reports[1].dlsr = 6;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(buf, &pos, &sr), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.sr_called);
	compare_rtcp_pkt_sender_report(&cap.sr, &sr);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_receiver_report_no_reports(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_receiver_report rr;
	struct rtcp_capture cap;

	memset(&rr, 0, sizeof(rr));
	rr.ssrc = 0x55667788;
	rr.report_count = 0;

	CU_ASSERT_EQUAL(rtcp_pkt_write_receiver_report(buf, &pos, &rr), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.rr_called);
	compare_rtcp_pkt_receiver_report(&cap.rr, &rr);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_receiver_report_max_reports(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_receiver_report rr;
	struct rtcp_capture cap;

	memset(&rr, 0, sizeof(rr));
	rr.ssrc = 0x55667788;
	rr.report_count = 31;

	for (uint32_t i = 0; i < rr.report_count; i++) {
		rr.reports[i].ssrc = i + 1;
		rr.reports[i].fraction = (uint8_t)i;
		rr.reports[i].lost = (int32_t)i - 15;
		rr.reports[i].ext_highest_seqnum = 1000 + i;
		rr.reports[i].jitter = 2000 + i;
		rr.reports[i].lsr.seconds = i;
		rr.reports[i].lsr.fraction = i * 2;
		rr.reports[i].dlsr = i * 3;
	}

	CU_ASSERT_EQUAL(rtcp_pkt_write_receiver_report(buf, &pos, &rr), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.rr_called);
	compare_rtcp_pkt_receiver_report(&cap.rr, &rr);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sdes_single_chunk_cname(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t cname_data[] = "abc";
	struct rtcp_pkt_sdes_item item;
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes sdes;
	struct rtcp_capture cap;

	memset(&item, 0, sizeof(item));
	item.type = RTCP_PKT_SDES_TYPE_CNAME;
	item.data = cname_data;
	item.data_len = sizeof(cname_data) - 1;

	memset(&chunk, 0, sizeof(chunk));
	chunk.ssrc = 0x1234;
	chunk.item_count = 1;
	chunk.items = &item;

	memset(&sdes, 0, sizeof(sdes));
	sdes.chunk_count = 1;
	sdes.chunks = &chunk;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, &sdes), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 1);
	CU_ASSERT_EQUAL(cap.sdes_items[0].ssrc, 0x1234);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.type, RTCP_PKT_SDES_TYPE_CNAME);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.data_len, 3);
	CU_ASSERT_EQUAL(memcmp(cap.sdes_items[0].item.data, "abc", 3), 0);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sdes_multi_item_chunk(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t cname_data[] = "abc";
	static const uint8_t name_data[] = "xyz";
	struct rtcp_pkt_sdes_item items[2];
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes sdes;
	struct rtcp_capture cap;

	memset(items, 0, sizeof(items));
	items[0].type = RTCP_PKT_SDES_TYPE_CNAME;
	items[0].data = cname_data;
	items[0].data_len = sizeof(cname_data) - 1;
	items[1].type = RTCP_PKT_SDES_TYPE_NAME;
	items[1].data = name_data;
	items[1].data_len = sizeof(name_data) - 1;

	memset(&chunk, 0, sizeof(chunk));
	chunk.ssrc = 0x1234;
	chunk.item_count = 2;
	chunk.items = items;

	memset(&sdes, 0, sizeof(sdes));
	sdes.chunk_count = 1;
	sdes.chunks = &chunk;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, &sdes), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 2);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.type, RTCP_PKT_SDES_TYPE_CNAME);
	CU_ASSERT_EQUAL(memcmp(cap.sdes_items[0].item.data, "abc", 3), 0);
	CU_ASSERT_EQUAL(cap.sdes_items[1].item.type, RTCP_PKT_SDES_TYPE_NAME);
	CU_ASSERT_EQUAL(memcmp(cap.sdes_items[1].item.data, "xyz", 3), 0);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sdes_priv_item(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t prefix[] = "px";
	static const uint8_t value[] = "val";
	struct rtcp_pkt_sdes_item item;
	struct rtcp_pkt_sdes_chunk chunk;
	struct rtcp_pkt_sdes sdes;
	struct rtcp_capture cap;

	memset(&item, 0, sizeof(item));
	item.type = RTCP_PKT_SDES_TYPE_PRIV;
	item.priv.prefix_len = sizeof(prefix) - 1;
	item.priv.prefix = prefix;
	item.priv.value_len = sizeof(value) - 1;
	item.priv.value = value;

	memset(&chunk, 0, sizeof(chunk));
	chunk.ssrc = 0x1234;
	chunk.item_count = 1;
	chunk.items = &item;

	memset(&sdes, 0, sizeof(sdes));
	sdes.chunk_count = 1;
	sdes.chunks = &chunk;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, &sdes), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 1);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.type, RTCP_PKT_SDES_TYPE_PRIV);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.priv.prefix_len, 2);
	CU_ASSERT_EQUAL(memcmp(cap.sdes_items[0].item.priv.prefix, "px", 2), 0);
	CU_ASSERT_EQUAL(cap.sdes_items[0].item.priv.value_len, 3);
	CU_ASSERT_EQUAL(memcmp(cap.sdes_items[0].item.priv.value, "val", 3), 0);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_sdes_multi_chunk(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t cname_data[] = "abc";
	struct rtcp_pkt_sdes_item item1;
	struct rtcp_pkt_sdes_item item2;
	struct rtcp_pkt_sdes_chunk chunks[2];
	struct rtcp_pkt_sdes sdes;
	struct rtcp_capture cap;

	memset(&item1, 0, sizeof(item1));
	item1.type = RTCP_PKT_SDES_TYPE_CNAME;
	item1.data = cname_data;
	item1.data_len = sizeof(cname_data) - 1;

	item2 = item1;

	memset(chunks, 0, sizeof(chunks));
	chunks[0].ssrc = 0x1111;
	chunks[0].item_count = 1;
	chunks[0].items = &item1;
	chunks[1].ssrc = 0x2222;
	chunks[1].item_count = 1;
	chunks[1].items = &item2;

	memset(&sdes, 0, sizeof(sdes));
	sdes.chunk_count = 2;
	sdes.chunks = chunks;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sdes(buf, &pos, &sdes), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 2);
	CU_ASSERT_EQUAL(cap.sdes_items[0].ssrc, 0x1111);
	CU_ASSERT_EQUAL(cap.sdes_items[1].ssrc, 0x2222);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_bye_no_reason(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_bye bye;
	struct rtcp_capture cap;

	memset(&bye, 0, sizeof(bye));
	bye.source_count = 2;
	bye.sources[0] = 0x11111111;
	bye.sources[1] = 0x22222222;
	bye.reason = NULL;
	bye.reason_len = 0;

	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(buf, &pos, &bye), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.bye_called);
	CU_ASSERT_EQUAL(cap.bye.reason_len, 0);
	compare_rtcp_pkt_bye(&cap.bye, &bye);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_bye_with_reason(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t reason[] = "bye bye";
	struct rtcp_pkt_bye bye;
	struct rtcp_capture cap;

	memset(&bye, 0, sizeof(bye));
	bye.source_count = 1;
	bye.sources[0] = 0xCAFEBABE;
	bye.reason = reason;
	bye.reason_len = sizeof(reason) - 1;

	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(buf, &pos, &bye), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.bye_called);
	CU_ASSERT_EQUAL(cap.bye.reason_len, 7);
	compare_rtcp_pkt_bye(&cap.bye, &bye);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_app_no_data(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_app app;
	struct rtcp_capture cap;

	memset(&app, 0, sizeof(app));
	app.ssrc = 0x11223344;
	app.name = 0x41424344;
	app.subtype = 3;
	app.data = NULL;
	app.data_len = 0;

	CU_ASSERT_EQUAL(rtcp_pkt_write_app(buf, &pos, &app), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.app_called);
	CU_ASSERT_EQUAL(cap.app.ssrc, app.ssrc);
	CU_ASSERT_EQUAL(cap.app.name, app.name);
	CU_ASSERT_EQUAL(cap.app.subtype, app.subtype);
	CU_ASSERT_EQUAL(cap.app.data_len, 0);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_roundtrip_app_with_data_padding_asymmetry(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	static const uint8_t data[] = "hello";
	struct rtcp_pkt_app app;
	struct rtcp_capture cap;

	memset(&app, 0, sizeof(app));
	app.ssrc = 0x11223344;
	app.name = 0x41424344;
	app.subtype = 1;
	app.data = data;
	app.data_len = sizeof(data) - 1; /* 5 */

	CU_ASSERT_EQUAL(rtcp_pkt_write_app(buf, &pos, &app), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.app_called);
	/* Documented read/write asymmetry: on read, data_len includes the
	 * 4-byte alignment padding (8 = 5 real bytes + 3 zero pad bytes),
	 * since the wire format carries no explicit unpadded length. */
	CU_ASSERT_EQUAL(cap.app.data_len, 8);
	CU_ASSERT_EQUAL(memcmp(cap.app.data, "hello", 5), 0);
	CU_ASSERT_EQUAL(cap.app.data[5], 0);
	CU_ASSERT_EQUAL(cap.app.data[6], 0);
	CU_ASSERT_EQUAL(cap.app.data[7], 0);

	pomp_buffer_unref(buf);
}


static void rtpfb_roundtrip(struct rtcp_pkt_rtpfb_report *report)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_capture cap;

	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(buf, &pos, report), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.rtpfb_called);
	compare_rtcp_pkt_rtpfb_report(&cap.rtpfb, report);

	pomp_buffer_unref(buf);
}


/* Hand-verified: single "small delta received" (symbol=1) entry encodes as
 * one run-length chunk of length 1. */
static void test_rtcp_pkt_rtpfb_hand_verified_run_len1(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_rtpfb_feedback feedbacks[1];
	struct rtcp_pkt_rtpfb_report report;
	const uint8_t *cdata = NULL;
	size_t len = 0;
	static const uint8_t expected[] = {
		0x8F, 0xCD, 0x00, 0x05, /* header: FMT=15, type=205, len=5 */
		0x11, 0x11, 0x11, 0x11, /* sender_ssrc */
		0x22, 0x22, 0x22, 0x22, /* media_ssrc */
		0x00, 0x64, /* base_seq = 100 */
		0x00, 0x01, /* status_count = 1 */
		0x00, 0x12, /* ref_time high 16 bits (0x001234 >> 8) */
		0x34, /* ref_time low byte */
		0x07, /* feedback_pkt_count */
		0x20, 0x01, /* run-length chunk: symbol=1, len=1 */
		0x05, /* recv_delta for the single entry */
		0x00, /* 4-byte alignment padding */
	};

	feedbacks[0].seq_num = 100;
	feedbacks[0].pkt_status_symbol = 1;
	feedbacks[0].recv_delta = 5;

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x11111111;
	report.media_ssrc = 0x22222222;
	report.base_seq = 100;
	report.status_count = 1;
	report.ref_time = 0x001234;
	report.feedback_pkt_count = 0x07;
	report.feedbacks = feedbacks;

	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(buf, &pos, &report), 0);
	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, sizeof(expected));
	CU_ASSERT_EQUAL(memcmp(cdata, expected, sizeof(expected)), 0);

	pomp_buffer_unref(buf);

	rtpfb_roundtrip(&report);
}


/* Hand-verified: two "not received" (symbol=0) entries, no recv_delta bytes
 * at all, encoded as one run-length chunk of length 2. */
static void test_rtcp_pkt_rtpfb_hand_verified_run_not_received_len2(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_rtpfb_feedback feedbacks[2];
	struct rtcp_pkt_rtpfb_report report;
	const uint8_t *cdata = NULL;
	size_t len = 0;
	static const uint8_t expected[] = {
		0x8F, 0xCD, 0x00, 0x05, /* header: FMT=15, type=205, len=5 */
		0x33, 0x33, 0x33, 0x33, /* sender_ssrc */
		0x44, 0x44, 0x44, 0x44, /* media_ssrc */
		0x00, 0x00, /* base_seq = 0 */
		0x00, 0x02, /* status_count = 2 */
		0x00, 0x00, /* ref_time high 16 bits */
		0x00, /* ref_time low byte */
		0x00, /* feedback_pkt_count */
		0x00, 0x02, /* run-length chunk: symbol=0, len=2 */
		0x00, 0x00, /* 4-byte alignment padding */
	};

	feedbacks[0].seq_num = 0;
	feedbacks[0].pkt_status_symbol = 0;
	feedbacks[0].recv_delta = 0;
	feedbacks[1].seq_num = 1;
	feedbacks[1].pkt_status_symbol = 0;
	feedbacks[1].recv_delta = 0;

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x33333333;
	report.media_ssrc = 0x44444444;
	report.base_seq = 0;
	report.status_count = 2;
	report.feedbacks = feedbacks;

	CU_ASSERT_EQUAL(rtcp_pkt_write_rtpfb(buf, &pos, &report), 0);
	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, sizeof(expected));
	CU_ASSERT_EQUAL(memcmp(cdata, expected, sizeof(expected)), 0);

	pomp_buffer_unref(buf);

	rtpfb_roundtrip(&report);
}


static void fill_rtpfb_constant(struct rtcp_pkt_rtpfb_feedback *fb,
				uint16_t count,
				uint8_t symbol,
				int16_t delta,
				uint16_t base_seq)
{
	for (uint16_t i = 0; i < count; i++) {
		fb[i].seq_num = (uint16_t)(base_seq + i);
		fb[i].pkt_status_symbol = symbol;
		fb[i].recv_delta = delta;
	}
}


static void test_rtcp_pkt_roundtrip_rtpfb_run_constant_len2(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[2];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_constant(feedbacks, 2, 1, 7, 0);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 2;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void test_rtcp_pkt_roundtrip_rtpfb_run_constant_len20(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[20];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_constant(feedbacks, 20, 1, 3, 1000);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 1000;
	report.status_count = 20;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void fill_rtpfb_alternating(struct rtcp_pkt_rtpfb_feedback *fb,
				   uint16_t count,
				   uint8_t sym_a,
				   uint8_t sym_b,
				   uint16_t base_seq)
{
	for (uint16_t i = 0; i < count; i++) {
		uint8_t symbol = (i % 2 == 0) ? sym_a : sym_b;
		fb[i].seq_num = (uint16_t)(base_seq + i);
		fb[i].pkt_status_symbol = symbol;
		/* Symbols 0 ("not received") and 3 carry no delta on the
		 * wire at all: rtcp_pkt_read_rtpfb() always decodes their
		 * recv_delta as 0, regardless of what was written. Match
		 * that here so the round trip compares equal. */
		fb[i].recv_delta =
			(symbol == 1 || symbol == 2) ? (int16_t)(i + 1) : 0;
	}
}


static void test_rtcp_pkt_roundtrip_rtpfb_status_vector_small_len14(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[14];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_alternating(feedbacks, 14, 0, 1, 0);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 14;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void
test_rtcp_pkt_roundtrip_rtpfb_status_vector_small_len15_spillover(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[15];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_alternating(feedbacks, 15, 0, 1, 0);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 15;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void test_rtcp_pkt_roundtrip_rtpfb_status_vector_large_len7(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[7];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_alternating(feedbacks, 7, 1, 2, 0);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 7;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void test_rtcp_pkt_roundtrip_rtpfb_status_vector_large_len8(void)
{
	struct rtcp_pkt_rtpfb_feedback feedbacks[8];
	struct rtcp_pkt_rtpfb_report report;

	fill_rtpfb_alternating(feedbacks, 8, 1, 2, 0);

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 8;
	report.feedbacks = feedbacks;

	rtpfb_roundtrip(&report);
}


static void test_rtcp_pkt_roundtrip_rtpfb_status_count_zero(void)
{
	struct rtcp_pkt_rtpfb_report report;

	memset(&report, 0, sizeof(report));
	report.sender_ssrc = 0x1;
	report.media_ssrc = 0x2;
	report.base_seq = 0;
	report.status_count = 0;
	report.feedbacks = NULL;

	rtpfb_roundtrip(&report);
}


static void test_rtcp_pkt_read_rtpfb_e2big_swallowed_by_dispatcher(void)
{
	/* Hand-built: declares status_count = RTPFB_MAX_PKT + 1 (2001) in
	 * the fixed section, with no chunk/delta bytes needed since
	 * rtcp_pkt_read_rtpfb() bails out with -E2BIG right after reading
	 * status_count. The outer rtcp_pkt_read() dispatcher discards that
	 * return value, so the overall call still returns 0 and the
	 * rtpfb_report callback never fires. */
	static const uint8_t data[] = {
		0x8F, 0xCD, 0x00, 0x04, /* header: FMT=15, type=205, len=4 */
		0x00, 0x00, 0x00, 0x01, /* sender_ssrc */
		0x00, 0x00, 0x00, 0x02, /* media_ssrc */
		0x00, 0x00, /* base_seq */
		0x07, 0xD1, /* status_count = 2001 */
		0x00, 0x00, 0x00, 0x00, /* ref_time + feedback_pkt_count */
	};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_FALSE(cap.rtpfb_called);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_einval_null_buf(void)
{
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(NULL, &k_full_cbs, &cap), -EINVAL);
}


static void test_rtcp_pkt_read_einval_null_cbs(void)
{
	static const uint8_t data[] = {0x80, 0xCA, 0x00, 0x00};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, NULL, NULL), -EINVAL);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_eio_too_short_for_header(void)
{
	static const uint8_t data[] = {0x80, 0xCA, 0x00};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), -EIO);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_eio_bad_version(void)
{
	/* version bits = 1 instead of 2 */
	static const uint8_t data[] = {0x40, 0xCA, 0x00, 0x00};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), -EIO);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_eio_declared_length_exceeds_buffer(void)
{
	/* header declares len=1 (8 bytes total) but buffer only has the
	 * 4-byte header itself */
	static const uint8_t data[] = {0x80, 0xC9, 0x00, 0x01};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), -EIO);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_unknown_type_no_callback_fires(void)
{
	/* type = 199, unused, no content */
	static const uint8_t data[] = {0x80, 0xC7, 0x00, 0x00};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_FALSE(cap.sr_called);
	CU_ASSERT_FALSE(cap.rr_called);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 0);
	CU_ASSERT_FALSE(cap.bye_called);
	CU_ASSERT_FALSE(cap.app_called);
	CU_ASSERT_FALSE(cap.rtpfb_called);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_compound_two_valid_packets(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(0);
	size_t pos = 0;
	struct rtcp_pkt_sender_report sr;
	struct rtcp_pkt_bye bye;
	struct rtcp_capture cap;

	memset(&sr, 0, sizeof(sr));
	sr.ssrc = 0x1;
	sr.ntp_timestamp.seconds = 1;
	sr.rtp_timestamp = 2;
	sr.sender_packet_count = 3;
	sr.sender_byte_count = 4;

	memset(&bye, 0, sizeof(bye));
	bye.source_count = 1;
	bye.sources[0] = 0x2;

	CU_ASSERT_EQUAL(rtcp_pkt_write_sender_report(buf, &pos, &sr), 0);
	CU_ASSERT_EQUAL(rtcp_pkt_write_bye(buf, &pos, &bye), 0);

	capture_reset(&cap);
	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_TRUE(cap.sr_called);
	CU_ASSERT_TRUE(cap.bye_called);
	compare_rtcp_pkt_sender_report(&cap.sr, &sr);
	compare_rtcp_pkt_bye(&cap.bye, &bye);

	pomp_buffer_unref(buf);
}


static void test_rtcp_pkt_read_resilience_malformed_sdes_then_valid_bye(void)
{
	/* Sub-packet 1: SDES, 1 chunk, 1 CNAME item whose declared data_len
	 * (200) deliberately exceeds the packet's own declared bound -- the
	 * internal read fails with -EIO before the sdes_item callback is
	 * invoked for this item, so it never fires. 2 filler zero bytes pad
	 * the sub-packet to its declared header.len (2 words = 8 bytes of
	 * content) so the next sub-packet starts exactly where declared.
	 * Sub-packet 2: a normal BYE (source_count=1, no reason). */
	static const uint8_t data[] = {
		/* malformed SDES: header + ssrc + item(type,bad data_len) +
		 * 2 filler bytes */
		0x81,
		0xCA,
		0x00,
		0x02,
		0xAA,
		0xAA,
		0xAA,
		0xAA,
		0x01,
		0xC8,
		0x00,
		0x00,
		/* valid BYE: header + 1 source */
		0x81,
		0xCB,
		0x00,
		0x01,
		0x12,
		0x34,
		0x56,
		0x78,
	};
	struct pomp_buffer *buf =
		make_pomp_buffer_from_bytes(data, sizeof(data));
	struct rtcp_capture cap;
	capture_reset(&cap);

	CU_ASSERT_EQUAL(rtcp_pkt_read(buf, &k_full_cbs, &cap), 0);
	CU_ASSERT_EQUAL(cap.sdes_item_count, 0);
	CU_ASSERT_TRUE(cap.bye_called);
	CU_ASSERT_EQUAL(cap.bye.source_count, 1);
	CU_ASSERT_EQUAL(cap.bye.sources[0], 0x12345678);
	CU_ASSERT_EQUAL(cap.bye.reason_len, 0);

	pomp_buffer_unref(buf);
}


CU_TestInfo s_test_rtcp_pkt[] = {
	{FN("type_str"), &test_rtcp_pkt_type_str},
	{FN("sdes_type_str"), &test_rtcp_pkt_sdes_type_str},
	{FN("write_sender_report_einval"),
	 &test_rtcp_pkt_write_sender_report_einval},
	{FN("write_receiver_report_einval"),
	 &test_rtcp_pkt_write_receiver_report_einval},
	{FN("write_sdes_einval"), &test_rtcp_pkt_write_sdes_einval},
	{FN("write_bye_einval"), &test_rtcp_pkt_write_bye_einval},
	{FN("write_app_einval"), &test_rtcp_pkt_write_app_einval},
	{FN("write_rtpfb_einval"), &test_rtcp_pkt_write_rtpfb_einval},
	{FN("write_sdes_priv_item_too_long"),
	 &test_rtcp_pkt_write_sdes_priv_item_too_long},
	{FN("roundtrip_sender_report_no_reports"),
	 &test_rtcp_pkt_roundtrip_sender_report_no_reports},
	{FN("roundtrip_sender_report_with_reports"),
	 &test_rtcp_pkt_roundtrip_sender_report_with_reports},
	{FN("roundtrip_receiver_report_no_reports"),
	 &test_rtcp_pkt_roundtrip_receiver_report_no_reports},
	{FN("roundtrip_receiver_report_max_reports"),
	 &test_rtcp_pkt_roundtrip_receiver_report_max_reports},
	{FN("roundtrip_sdes_single_chunk_cname"),
	 &test_rtcp_pkt_roundtrip_sdes_single_chunk_cname},
	{FN("roundtrip_sdes_multi_item_chunk"),
	 &test_rtcp_pkt_roundtrip_sdes_multi_item_chunk},
	{FN("roundtrip_sdes_priv_item"),
	 &test_rtcp_pkt_roundtrip_sdes_priv_item},
	{FN("roundtrip_sdes_multi_chunk"),
	 &test_rtcp_pkt_roundtrip_sdes_multi_chunk},
	{FN("roundtrip_bye_no_reason"), &test_rtcp_pkt_roundtrip_bye_no_reason},
	{FN("roundtrip_bye_with_reason"),
	 &test_rtcp_pkt_roundtrip_bye_with_reason},
	{FN("roundtrip_app_no_data"), &test_rtcp_pkt_roundtrip_app_no_data},
	{FN("roundtrip_app_with_data_padding_asymmetry"),
	 &test_rtcp_pkt_roundtrip_app_with_data_padding_asymmetry},
	{FN("rtpfb_hand_verified_run_len1"),
	 &test_rtcp_pkt_rtpfb_hand_verified_run_len1},
	{FN("rtpfb_hand_verified_run_not_received_len2"),
	 &test_rtcp_pkt_rtpfb_hand_verified_run_not_received_len2},
	{FN("roundtrip_rtpfb_run_constant_len2"),
	 &test_rtcp_pkt_roundtrip_rtpfb_run_constant_len2},
	{FN("roundtrip_rtpfb_run_constant_len20"),
	 &test_rtcp_pkt_roundtrip_rtpfb_run_constant_len20},
	{FN("roundtrip_rtpfb_status_vector_small_len14"),
	 &test_rtcp_pkt_roundtrip_rtpfb_status_vector_small_len14},
	{FN("roundtrip_rtpfb_status_vector_small_len15_spillover"),
	 &test_rtcp_pkt_roundtrip_rtpfb_status_vector_small_len15_spillover},
	{FN("roundtrip_rtpfb_status_vector_large_len7"),
	 &test_rtcp_pkt_roundtrip_rtpfb_status_vector_large_len7},
	{FN("roundtrip_rtpfb_status_vector_large_len8"),
	 &test_rtcp_pkt_roundtrip_rtpfb_status_vector_large_len8},
	{FN("roundtrip_rtpfb_status_count_zero"),
	 &test_rtcp_pkt_roundtrip_rtpfb_status_count_zero},
	{FN("read_rtpfb_e2big_swallowed_by_dispatcher"),
	 &test_rtcp_pkt_read_rtpfb_e2big_swallowed_by_dispatcher},
	{FN("read_einval_null_buf"), &test_rtcp_pkt_read_einval_null_buf},
	{FN("read_einval_null_cbs"), &test_rtcp_pkt_read_einval_null_cbs},
	{FN("read_eio_too_short_for_header"),
	 &test_rtcp_pkt_read_eio_too_short_for_header},
	{FN("read_eio_bad_version"), &test_rtcp_pkt_read_eio_bad_version},
	{FN("read_eio_declared_length_exceeds_buffer"),
	 &test_rtcp_pkt_read_eio_declared_length_exceeds_buffer},
	{FN("read_unknown_type_no_callback_fires"),
	 &test_rtcp_pkt_read_unknown_type_no_callback_fires},
	{FN("read_compound_two_valid_packets"),
	 &test_rtcp_pkt_read_compound_two_valid_packets},
	{FN("read_resilience_malformed_sdes_then_valid_bye"),
	 &test_rtcp_pkt_read_resilience_malformed_sdes_then_valid_bye},
	CU_TEST_INFO_NULL,
};
