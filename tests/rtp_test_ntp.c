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

#include "rtp/ntp.h"


/* Ported from the old tests/test_rtp_ntp.c table, now with real assertions */
static void test_ntp_timestamp64_diff_us(void)
{
	int64_t r = 0;
	int res;
	static const struct {
		struct ntp_timestamp64 t1;
		struct ntp_timestamp64 t2;
		int64_t expected;
	} table[] = {
		{{1, 0x40000000}, {1, 0x20000000}, 125000},
		{{1, 0x20000000}, {1, 0x40000000}, -125000},
		{{5, 0x40000000}, {2, 0x20000000}, 3125000},
		{{5, 0x20000000}, {2, 0x40000000}, 2875000},
		{{2, 0x40000000}, {5, 0x20000000}, -2875000},
		{{2, 0x20000000}, {5, 0x40000000}, -3125000},
		{{5, 0}, {5, 0}, 0},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		res = ntp_timestamp64_diff_us(&table[i].t1, &table[i].t2, &r);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(r, table[i].expected);
	}

	CU_ASSERT_EQUAL(ntp_timestamp64_diff_us(NULL, &table[0].t2, &r),
			-EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_diff_us(&table[0].t1, NULL, &r),
			-EINVAL);
	CU_ASSERT_EQUAL(
		ntp_timestamp64_diff_us(&table[0].t1, &table[0].t2, NULL),
		-EINVAL);
}


static void test_ntp_timestamp64_to_us(void)
{
	uint64_t us = 0;
	static const struct {
		struct ntp_timestamp64 t;
		uint64_t expected;
	} table[] = {
		{{0, 0}, 0},
		{{1, 0}, 1000000},
		{{0, 0x80000000}, 500000},
		{{1, 0x80000000}, 1500000},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_EQUAL(ntp_timestamp64_to_us(&table[i].t, &us), 0);
		CU_ASSERT_EQUAL(us, table[i].expected);
	}

	CU_ASSERT_EQUAL(ntp_timestamp64_to_us(NULL, &us), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_to_us(&table[0].t, NULL), -EINVAL);
}


static void test_ntp_timestamp64_from_us(void)
{
	struct ntp_timestamp64 t;
	static const struct {
		uint64_t us;
		struct ntp_timestamp64 expected;
	} table[] = {
		{0, {0, 0}},
		{1000000, {1, 0}},
		{500000, {0, 0x80000000}},
		{1500000, {1, 0x80000000}},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		memset(&t, 0xff, sizeof(t));
		CU_ASSERT_EQUAL(ntp_timestamp64_from_us(&t, table[i].us), 0);
		CU_ASSERT_EQUAL(t.seconds, table[i].expected.seconds);
		CU_ASSERT_EQUAL(t.fraction, table[i].expected.fraction);
	}

	CU_ASSERT_EQUAL(ntp_timestamp64_from_us(NULL, 0), -EINVAL);
}


static void test_ntp_timestamp64_timespec(void)
{
	struct ntp_timestamp64 t = {3, 0x80000000};
	struct ntp_timestamp64 t2;
	struct timespec ts;

	CU_ASSERT_EQUAL(ntp_timestamp64_to_timespec(&t, &ts), 0);
	CU_ASSERT_EQUAL(ts.tv_sec, 3);
	CU_ASSERT_EQUAL(ts.tv_nsec, 500000000);

	CU_ASSERT_EQUAL(ntp_timestamp64_from_timespec(&t2, &ts), 0);
	CU_ASSERT_EQUAL(t2.seconds, t.seconds);
	CU_ASSERT_EQUAL(t2.fraction, t.fraction);

	CU_ASSERT_EQUAL(ntp_timestamp64_to_timespec(NULL, &ts), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_to_timespec(&t, NULL), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_from_timespec(NULL, &ts), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_from_timespec(&t2, NULL), -EINVAL);
}


static void test_ntp_timestamp64_to_ntp_timestamp32(void)
{
	struct ntp_timestamp64 t1 = {0x00011234, 0xABCD1234};
	struct ntp_timestamp32 t2;

	CU_ASSERT_EQUAL(ntp_timestamp64_to_ntp_timestamp32(&t1, &t2), 0);
	CU_ASSERT_EQUAL(t2.seconds, 0x1234);
	CU_ASSERT_EQUAL(t2.fraction, 0xABCD);

	CU_ASSERT_EQUAL(ntp_timestamp64_to_ntp_timestamp32(NULL, &t2), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp64_to_ntp_timestamp32(&t1, NULL), -EINVAL);
}


static void test_ntp_timestamp64_32_roundtrip_lossy(void)
{
	/* Documents that converting 64->32->64 loses the low 16 bits of the
	 * fraction: this is an expected precision limit, not a bug. */
	struct ntp_timestamp64 orig = {100, 0x1234ABCD};
	struct ntp_timestamp32 t32;
	struct ntp_timestamp64 back;

	CU_ASSERT_EQUAL(ntp_timestamp64_to_ntp_timestamp32(&orig, &t32), 0);
	CU_ASSERT_EQUAL(ntp_timestamp32_to_ntp_timestamp64(&t32, &back), 0);

	CU_ASSERT_EQUAL(back.seconds, orig.seconds);
	CU_ASSERT_NOT_EQUAL(back.fraction, orig.fraction);
	CU_ASSERT_EQUAL(back.fraction, 0x12340000);
}


static void test_ntp_timestamp32_diff_us(void)
{
	int64_t r = 0;
	static const struct {
		struct ntp_timestamp32 t1;
		struct ntp_timestamp32 t2;
		int64_t expected;
	} table[] = {
		{{1, 0x4000}, {1, 0x2000}, 125000},
		{{1, 0x2000}, {1, 0x4000}, -125000},
		{{5, 0x4000}, {2, 0x2000}, 3125000},
		{{5, 0x2000}, {2, 0x4000}, 2875000},
		{{2, 0x4000}, {5, 0x2000}, -2875000},
		{{2, 0x2000}, {5, 0x4000}, -3125000},
		{{5, 0}, {5, 0}, 0},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_EQUAL(
			ntp_timestamp32_diff_us(&table[i].t1, &table[i].t2, &r),
			0);
		CU_ASSERT_EQUAL(r, table[i].expected);
	}

	CU_ASSERT_EQUAL(ntp_timestamp32_diff_us(NULL, &table[0].t2, &r),
			-EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_diff_us(&table[0].t1, NULL, &r),
			-EINVAL);
	CU_ASSERT_EQUAL(
		ntp_timestamp32_diff_us(&table[0].t1, &table[0].t2, NULL),
		-EINVAL);
}


static void test_ntp_timestamp32_to_us(void)
{
	uint64_t us = 0;
	static const struct {
		struct ntp_timestamp32 t;
		uint64_t expected;
	} table[] = {
		{{0, 0}, 0},
		{{1, 0}, 1000000},
		{{0, 0x8000}, 500000},
		{{1, 0x8000}, 1500000},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_EQUAL(ntp_timestamp32_to_us(&table[i].t, &us), 0);
		CU_ASSERT_EQUAL(us, table[i].expected);
	}

	CU_ASSERT_EQUAL(ntp_timestamp32_to_us(NULL, &us), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_to_us(&table[0].t, NULL), -EINVAL);
}


static void test_ntp_timestamp32_from_us(void)
{
	struct ntp_timestamp32 t;
	static const struct {
		uint64_t us;
		struct ntp_timestamp32 expected;
	} table[] = {
		{0, {0, 0}},
		{1000000, {1, 0}},
		{500000, {0, 0x8000}},
		{1500000, {1, 0x8000}},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		memset(&t, 0xff, sizeof(t));
		CU_ASSERT_EQUAL(ntp_timestamp32_from_us(&t, table[i].us), 0);
		CU_ASSERT_EQUAL(t.seconds, table[i].expected.seconds);
		CU_ASSERT_EQUAL(t.fraction, table[i].expected.fraction);
	}

	CU_ASSERT_EQUAL(ntp_timestamp32_from_us(NULL, 0), -EINVAL);
}


static void test_ntp_timestamp32_timespec(void)
{
	struct ntp_timestamp32 t = {3, 0x8000};
	struct ntp_timestamp32 t2;
	struct timespec ts;

	CU_ASSERT_EQUAL(ntp_timestamp32_to_timespec(&t, &ts), 0);
	CU_ASSERT_EQUAL(ts.tv_sec, 3);
	CU_ASSERT_EQUAL(ts.tv_nsec, 500000000);

	CU_ASSERT_EQUAL(ntp_timestamp32_from_timespec(&t2, &ts), 0);
	CU_ASSERT_EQUAL(t2.seconds, t.seconds);
	CU_ASSERT_EQUAL(t2.fraction, t.fraction);

	CU_ASSERT_EQUAL(ntp_timestamp32_to_timespec(NULL, &ts), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_to_timespec(&t, NULL), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_from_timespec(NULL, &ts), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_from_timespec(&t2, NULL), -EINVAL);
}


static void test_ntp_timestamp32_to_ntp_timestamp64(void)
{
	struct ntp_timestamp32 t1 = {0x1234, 0x5678};
	struct ntp_timestamp64 t2;

	CU_ASSERT_EQUAL(ntp_timestamp32_to_ntp_timestamp64(&t1, &t2), 0);
	CU_ASSERT_EQUAL(t2.seconds, 0x1234);
	CU_ASSERT_EQUAL(t2.fraction, 0x56780000);

	CU_ASSERT_EQUAL(ntp_timestamp32_to_ntp_timestamp64(NULL, &t2), -EINVAL);
	CU_ASSERT_EQUAL(ntp_timestamp32_to_ntp_timestamp64(&t1, NULL), -EINVAL);
}


CU_TestInfo s_test_ntp[] = {
	{FN("timestamp64_diff_us"), &test_ntp_timestamp64_diff_us},
	{FN("timestamp64_to_us"), &test_ntp_timestamp64_to_us},
	{FN("timestamp64_from_us"), &test_ntp_timestamp64_from_us},
	{FN("timestamp64_timespec"), &test_ntp_timestamp64_timespec},
	{FN("timestamp64_to_ntp_timestamp32"),
	 &test_ntp_timestamp64_to_ntp_timestamp32},
	{FN("timestamp64_32_roundtrip_lossy"),
	 &test_ntp_timestamp64_32_roundtrip_lossy},
	{FN("timestamp32_diff_us"), &test_ntp_timestamp32_diff_us},
	{FN("timestamp32_to_us"), &test_ntp_timestamp32_to_us},
	{FN("timestamp32_from_us"), &test_ntp_timestamp32_from_us},
	{FN("timestamp32_timespec"), &test_ntp_timestamp32_timespec},
	{FN("timestamp32_to_ntp_timestamp64"),
	 &test_ntp_timestamp32_to_ntp_timestamp64},
	CU_TEST_INFO_NULL,
};
