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

#include "rtp/rtp_jitter.h"
#include "rtp/rtp_pkt.h"


#define MAX_DELIVERED 64


struct delivered_pkt {
	uint16_t seqnum;
	uint32_t gap;
};


struct jitter_test_userdata {
	struct delivered_pkt items[MAX_DELIVERED];
	size_t count;
};


static void test_process_pkt_cb(struct rtp_jitter *jitter,
				const struct rtp_pkt *pkt,
				uint32_t gap,
				void *userdata)
{
	struct jitter_test_userdata *ud = userdata;
	(void)jitter;

	if (ud->count < MAX_DELIVERED) {
		ud->items[ud->count].seqnum = pkt->header.seqnum;
		ud->items[ud->count].gap = gap;
		ud->count++;
	}
}


/* Note: once successfully enqueued (accepted, dropped-as-old, or
 * dropped-as-duplicate), librtp itself destroys the packet -- callers must
 * not separately destroy a packet that was passed to rtp_jitter_enqueue(). */
static struct rtp_pkt *
make_test_pkt(uint16_t seqnum, uint64_t in_timestamp, uint64_t rtp_timestamp)
{
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL_FATAL(rtp_pkt_new(&pkt), 0);
	pkt->header.seqnum = seqnum;
	pkt->in_timestamp = in_timestamp;
	pkt->rtp_timestamp = rtp_timestamp;

	return pkt;
}


static void test_rtp_jitter_new_einval(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cfg cfg_no_clk = {0, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};

	CU_ASSERT_EQUAL(rtp_jitter_new(NULL, &cbs, NULL, &jitter), -EINVAL);
	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, NULL, NULL, &jitter), -EINVAL);
	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, NULL, NULL), -EINVAL);
	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg_no_clk, &cbs, NULL, &jitter),
			-EINVAL);
}


static void test_rtp_jitter_new_ok(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, NULL, &jitter), 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(jitter);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_get_info_einval(void)
{
	uint32_t clk_rate = 0;

	CU_ASSERT_EQUAL(rtp_jitter_get_info(NULL, &clk_rate, NULL, NULL),
			-EINVAL);
}


static void test_rtp_jitter_get_info_null_output_ptrs_ok(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	uint32_t clk_rate = 0;
	uint32_t jitter_avg = 0;
	int64_t skew_avg = 0;

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, NULL, &jitter), 0);

	CU_ASSERT_EQUAL(
		rtp_jitter_get_info(jitter, NULL, &jitter_avg, &skew_avg), 0);
	CU_ASSERT_EQUAL(rtp_jitter_get_info(jitter, &clk_rate, NULL, &skew_avg),
			0);
	CU_ASSERT_EQUAL(
		rtp_jitter_get_info(jitter, &clk_rate, &jitter_avg, NULL), 0);
	CU_ASSERT_EQUAL(rtp_jitter_get_info(jitter, NULL, NULL, NULL), 0);
	CU_ASSERT_EQUAL(clk_rate, 90000);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_enqueue_einval(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct rtp_pkt *pkt = NULL;

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, NULL, &jitter), 0);

	pkt = make_test_pkt(1, 1000, 90);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(NULL, pkt), -EINVAL);
	rtp_pkt_destroy(pkt); /* not consumed: self was NULL */

	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, NULL), -EINVAL);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_process_einval(void)
{
	CU_ASSERT_EQUAL(rtp_jitter_process(NULL, 0), -EINVAL);
}


static void test_rtp_jitter_in_order_delivery(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	for (uint16_t i = 0; i < 4; i++) {
		struct rtp_pkt *pkt = make_test_pkt(
			i, 1000000ULL * (i + 1), 90000ULL * (i + 1));
		CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt), 0);
	}
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);

	CU_ASSERT_EQUAL(ud.count, 4);
	for (uint16_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ud.items[i].seqnum, i);
		CU_ASSERT_EQUAL(ud.items[i].gap, 0);
	}

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_out_of_order_reorder(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	static const uint16_t arrival_order[] = {0, 2, 1, 3};
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	for (uint32_t i = 0; i < ARRAY_SIZE(arrival_order); i++) {
		uint16_t seqnum = arrival_order[i];
		struct rtp_pkt *pkt = make_test_pkt(seqnum,
						    1000000ULL * (seqnum + 1),
						    90000ULL * (seqnum + 1));
		CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt), 0);
	}
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);

	CU_ASSERT_EQUAL(ud.count, 4);
	for (uint16_t i = 0; i < 4; i++) {
		CU_ASSERT_EQUAL(ud.items[i].seqnum, i);
		CU_ASSERT_EQUAL(ud.items[i].gap, 0);
	}

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_gap_detection_on_loss(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	struct rtp_pkt *pkt0 = NULL;
	struct rtp_pkt *pkt2 = NULL;
	uint64_t pkt2_in_timestamp = 3000000;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	pkt0 = make_test_pkt(0, 1000000, 90000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt0), 0);
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);
	CU_ASSERT_EQUAL(ud.count, 1);

	/* seqnum 1 is never received */
	pkt2 = make_test_pkt(2, pkt2_in_timestamp, 180000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt2), 0);
	CU_ASSERT_EQUAL(
		rtp_jitter_process(jitter,
				   pkt2_in_timestamp + cfg.delay + 5000000),
		0);

	CU_ASSERT_EQUAL(ud.count, 2);
	CU_ASSERT_EQUAL(ud.items[1].seqnum, 2);
	CU_ASSERT_EQUAL(ud.items[1].gap, 1);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_drop_old_duplicate(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	struct rtp_pkt *pkt0 = NULL;
	struct rtp_pkt *dup = NULL;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	pkt0 = make_test_pkt(0, 1000000, 90000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt0), 0);
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);
	CU_ASSERT_EQUAL(ud.count, 1);

	dup = make_test_pkt(0, 1100000, 90000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, dup), 0);
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);
	CU_ASSERT_EQUAL(ud.count, 1);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_drop_exact_duplicate_in_queue(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	struct rtp_pkt *pkt_a = NULL;
	struct rtp_pkt *pkt_b = NULL;
	uint64_t in_ts = 1000000;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);
	CU_ASSERT_EQUAL(rtp_jitter_clear(jitter, 3), 0);

	pkt_a = make_test_pkt(5, in_ts, 450000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt_a), 0);
	pkt_b = make_test_pkt(5, in_ts + 100000, 450000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt_b), 0);

	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, in_ts + cfg.delay + 5000000),
			0);

	CU_ASSERT_EQUAL(ud.count, 1);
	CU_ASSERT_EQUAL(ud.items[0].seqnum, 5);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_process_stops_at_first_ineligible(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	struct rtp_pkt *pkt1 = NULL;
	uint64_t in_ts = 1000000;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	/* seqnum 0 never arrives */
	pkt1 = make_test_pkt(1, in_ts, 90000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt1), 0);

	/* Not yet past its playout deadline: nothing delivered */
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, in_ts), 0);
	CU_ASSERT_EQUAL(ud.count, 0);

	/* Past its deadline: delivered, with a detected gap of 1 */
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, in_ts + cfg.delay + 5000000),
			0);
	CU_ASSERT_EQUAL(ud.count, 1);
	CU_ASSERT_EQUAL(ud.items[0].seqnum, 1);
	CU_ASSERT_EQUAL(ud.items[0].gap, 1);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_clear_resets_state(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	struct rtp_pkt *pkt_a = NULL;
	struct rtp_pkt *pkt_b = NULL;
	struct rtp_pkt *pkt_fresh = NULL;
	uint32_t jitter_avg = 0;
	int64_t skew_avg = 0;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	pkt_a = make_test_pkt(10, 1000000, 900000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt_a), 0);
	pkt_b = make_test_pkt(11, 1050000, 990000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt_b), 0);

	CU_ASSERT_EQUAL(rtp_jitter_clear(jitter, 100), 0);
	CU_ASSERT_EQUAL(
		rtp_jitter_get_info(jitter, NULL, &jitter_avg, &skew_avg), 0);
	CU_ASSERT_EQUAL(jitter_avg, 0);
	CU_ASSERT_EQUAL(skew_avg, 0);

	pkt_fresh = make_test_pkt(100, 2000000, 900000);
	CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt_fresh), 0);
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);

	CU_ASSERT_EQUAL(ud.count, 1);
	CU_ASSERT_EQUAL(ud.items[0].seqnum, 100);
	CU_ASSERT_EQUAL(ud.items[0].gap, 0);

	rtp_jitter_destroy(jitter);
}


static void test_rtp_jitter_destroy_flushes_queue(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	for (uint16_t i = 0; i < 3; i++) {
		struct rtp_pkt *pkt = make_test_pkt(
			(uint16_t)(i + 10), 1000000ULL * (i + 1), 90000);
		CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt), 0);
	}

	/* Left unprocessed on purpose: destroy must flush the queue without
	 * crashing. For full confidence this should also be exercised under
	 * valgrind/ASan, which CUnit alone cannot substitute for. */
	CU_ASSERT_EQUAL(rtp_jitter_destroy(jitter), 0);
}


static void test_rtp_jitter_jitter_and_skew_avg_regular_stream(void)
{
	struct rtp_jitter *jitter = NULL;
	struct rtp_jitter_cfg cfg = {90000, 100000};
	struct rtp_jitter_cbs cbs = {.process_pkt = test_process_pkt_cb};
	struct jitter_test_userdata ud;
	uint32_t jitter_avg = 0;
	int64_t skew_avg = 0;
	memset(&ud, 0, sizeof(ud));

	CU_ASSERT_EQUAL(rtp_jitter_new(&cfg, &cbs, &ud, &jitter), 0);

	/* Perfectly regular stream: 1s per packet in both clocks, so
	 * delta_rx - delta_rtp(us) is exactly 0 every step. */
	for (uint16_t i = 0; i < 20; i++) {
		struct rtp_pkt *pkt = make_test_pkt(
			i, 1000000ULL * (i + 1), 90000ULL * (i + 1));
		CU_ASSERT_EQUAL(rtp_jitter_enqueue(jitter, pkt), 0);
	}
	CU_ASSERT_EQUAL(rtp_jitter_process(jitter, 0), 0);
	CU_ASSERT_EQUAL(ud.count, 20);

	CU_ASSERT_EQUAL(
		rtp_jitter_get_info(jitter, NULL, &jitter_avg, &skew_avg), 0);

	/* All-zero jitter samples keep the integer EMA exactly at 0 */
	CU_ASSERT_EQUAL(jitter_avg, 0);
	/* skew_avg's windowed-min/parabolic-blend/EMA formula is too
	 * intricate to hand-replicate reliably: assert only a tolerance
	 * band around zero, not an exact value. */
	CU_ASSERT(skew_avg > -1000 && skew_avg < 1000);

	rtp_jitter_destroy(jitter);
}


CU_TestInfo s_test_rtp_jitter[] = {
	{FN("new_einval"), &test_rtp_jitter_new_einval},
	{FN("new_ok"), &test_rtp_jitter_new_ok},
	{FN("get_info_einval"), &test_rtp_jitter_get_info_einval},
	{FN("get_info_null_output_ptrs_ok"),
	 &test_rtp_jitter_get_info_null_output_ptrs_ok},
	{FN("enqueue_einval"), &test_rtp_jitter_enqueue_einval},
	{FN("process_einval"), &test_rtp_jitter_process_einval},
	{FN("in_order_delivery"), &test_rtp_jitter_in_order_delivery},
	{FN("out_of_order_reorder"), &test_rtp_jitter_out_of_order_reorder},
	{FN("gap_detection_on_loss"), &test_rtp_jitter_gap_detection_on_loss},
	{FN("drop_old_duplicate"), &test_rtp_jitter_drop_old_duplicate},
	{FN("drop_exact_duplicate_in_queue"),
	 &test_rtp_jitter_drop_exact_duplicate_in_queue},
	{FN("process_stops_at_first_ineligible"),
	 &test_rtp_jitter_process_stops_at_first_ineligible},
	{FN("clear_resets_state"), &test_rtp_jitter_clear_resets_state},
	{FN("destroy_flushes_queue"), &test_rtp_jitter_destroy_flushes_queue},
	{FN("jitter_and_skew_avg_regular_stream"),
	 &test_rtp_jitter_jitter_and_skew_avg_regular_stream},
	CU_TEST_INFO_NULL,
};
