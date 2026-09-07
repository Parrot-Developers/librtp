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


static void test_timestamp_to_us(void)
{
	static const struct {
		uint64_t rtp_timestamp;
		uint32_t clk_rate;
		uint64_t expected;
	} table[] = {
		{0, 90000, 0},
		{90000, 90000, 1000000},
		{45000, 90000, 500000},
		{1, 90000, 11},
		{1, 0, 0},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		uint64_t us = rtp_timestamp_to_us(table[i].rtp_timestamp,
						  table[i].clk_rate);
		CU_ASSERT_EQUAL(us, table[i].expected);
	}
}


static void test_timestamp_from_us(void)
{
	static const struct {
		uint64_t us;
		uint32_t clk_rate;
		uint64_t expected;
	} table[] = {
		{0, 90000, 0},
		{1000000, 90000, 90000},
		{500000, 90000, 45000},
		/* clk_rate == 0: numerator is always 500000, so result is
		 * always 0 regardless of us */
		{1000, 0, 0},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		uint64_t rtp_timestamp =
			rtp_timestamp_from_us(table[i].us, table[i].clk_rate);
		CU_ASSERT_EQUAL(rtp_timestamp, table[i].expected);
	}
}


static void test_timestamp_roundtrip_tolerance(void)
{
	static const uint32_t clk_rates[] = {8000, 16000, 48000, 90000};
	static const uint64_t timestamps[] = {0, 1, 100, 12345, 1000000};

	for (uint32_t i = 0; i < ARRAY_SIZE(clk_rates); i++) {
		for (uint32_t j = 0; j < ARRAY_SIZE(timestamps); j++) {
			uint32_t clk_rate = clk_rates[i];
			uint64_t us =
				rtp_timestamp_to_us(timestamps[j], clk_rate);
			uint64_t back = rtp_timestamp_from_us(us, clk_rate);
			int64_t delta = (int64_t)back - (int64_t)timestamps[j];

			CU_ASSERT(delta >= -1 && delta <= 1);
		}
	}
}


CU_TestInfo s_test_timestamp[] = {
	{FN("to_us"), &test_timestamp_to_us},
	{FN("from_us"), &test_timestamp_from_us},
	{FN("roundtrip_tolerance"), &test_timestamp_roundtrip_tolerance},
	CU_TEST_INFO_NULL,
};
