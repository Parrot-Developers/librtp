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

#include "rtp_priv.h"


static void test_priv_diff_seqnum(void)
{
	static const struct {
		uint16_t sq1;
		uint16_t sq2;
		int16_t expected;
	} table[] = {
		{0, 0, 0},
		{1, 0, 1},
		{0, 1, -1},
		{100, 100, 0},
		{0, 65535, 1},
		{65535, 0, -1},
		{100, 65436, 200},
		{65436, 100, -200},
		{32767, 0, 32767},
		{0, 32767, -32767},
		{32768, 0, -32768},
		{0, 32768, -32768},
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(table); i++) {
		CU_ASSERT_EQUAL(rtp_diff_seqnum(table[i].sq1, table[i].sq2),
				table[i].expected);
	}
}


static void test_priv_write_read_u8(void)
{
	static const uint8_t values[] = {0x00, 0x7F, 0xFF};

	for (uint32_t i = 0; i < ARRAY_SIZE(values); i++) {
		struct pomp_buffer *buf = make_pomp_buffer(1);
		size_t pos = 0;
		uint8_t v = 0;

		CU_ASSERT_EQUAL(rtp_write_u8(buf, &pos, values[i]), 0);
		CU_ASSERT_EQUAL(pos, 1);

		pos = 0;
		CU_ASSERT_EQUAL(rtp_read_u8(buf, &pos, &v), 0);
		CU_ASSERT_EQUAL(v, values[i]);
		CU_ASSERT_EQUAL(pos, 1);

		pomp_buffer_unref(buf);
	}
}


static void test_priv_write_read_u16_network_order(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(2);
	size_t pos = 0;
	const uint8_t *cdata = NULL;
	size_t len = 0;
	uint16_t v = 0;

	CU_ASSERT_EQUAL(rtp_write_u16(buf, &pos, 0x1234), 0);
	CU_ASSERT_EQUAL(pos, 2);

	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, 2);
	CU_ASSERT_EQUAL(cdata[0], 0x12);
	CU_ASSERT_EQUAL(cdata[1], 0x34);

	pos = 0;
	CU_ASSERT_EQUAL(rtp_read_u16(buf, &pos, &v), 0);
	CU_ASSERT_EQUAL(v, 0x1234);
	CU_ASSERT_EQUAL(pos, 2);

	pomp_buffer_unref(buf);
}


static void test_priv_write_read_u32_network_order(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(4);
	size_t pos = 0;
	const uint8_t *cdata = NULL;
	size_t len = 0;
	uint32_t v = 0;

	CU_ASSERT_EQUAL(rtp_write_u32(buf, &pos, 0x12345678), 0);
	CU_ASSERT_EQUAL(pos, 4);

	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, 4);
	CU_ASSERT_EQUAL(cdata[0], 0x12);
	CU_ASSERT_EQUAL(cdata[1], 0x34);
	CU_ASSERT_EQUAL(cdata[2], 0x56);
	CU_ASSERT_EQUAL(cdata[3], 0x78);

	pos = 0;
	CU_ASSERT_EQUAL(rtp_read_u32(buf, &pos, &v), 0);
	CU_ASSERT_EQUAL(v, 0x12345678);
	CU_ASSERT_EQUAL(pos, 4);

	pomp_buffer_unref(buf);
}


static void test_priv_write_read_u64_network_order(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(8);
	size_t pos = 0;
	const uint8_t *cdata = NULL;
	size_t len = 0;
	uint64_t v = 0;
	static const uint8_t expected[8] = {
		0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

	CU_ASSERT_EQUAL(rtp_write_u64(buf, &pos, 0x0102030405060708ULL), 0);
	CU_ASSERT_EQUAL(pos, 8);

	CU_ASSERT_EQUAL(
		pomp_buffer_get_cdata(buf, (const void **)&cdata, &len, NULL),
		0);
	CU_ASSERT_EQUAL(len, 8);
	CU_ASSERT_EQUAL(memcmp(cdata, expected, 8), 0);

	pos = 0;
	CU_ASSERT_EQUAL(rtp_read_u64(buf, &pos, &v), 0);
	CU_ASSERT_EQUAL(v, 0x0102030405060708ULL);
	CU_ASSERT_EQUAL(pos, 8);

	pomp_buffer_unref(buf);
}


static void test_priv_read_insufficient_buffer(void)
{
	static const uint8_t one_byte[] = {0xAA};
	struct pomp_buffer *buf = make_pomp_buffer_from_bytes(one_byte, 1);
	size_t pos = 0;
	uint16_t v16 = 0;
	uint32_t v32 = 0;
	uint64_t v64 = 0;

	CU_ASSERT_EQUAL(rtp_read_u16(buf, &pos, &v16), -EINVAL);
	CU_ASSERT_EQUAL(pos, 0);

	CU_ASSERT_EQUAL(rtp_read_u32(buf, &pos, &v32), -EINVAL);
	CU_ASSERT_EQUAL(pos, 0);

	CU_ASSERT_EQUAL(rtp_read_u64(buf, &pos, &v64), -EINVAL);
	CU_ASSERT_EQUAL(pos, 0);

	pomp_buffer_unref(buf);
}


static void test_priv_write_sequence_advances_pos(void)
{
	struct pomp_buffer *buf = make_pomp_buffer(16);
	size_t pos = 0;
	size_t read_pos = 0;
	uint8_t v8 = 0;
	uint16_t v16 = 0;
	uint32_t v32 = 0;

	CU_ASSERT_EQUAL(rtp_write_u8(buf, &pos, 0xAA), 0);
	CU_ASSERT_EQUAL(pos, 1);
	CU_ASSERT_EQUAL(rtp_write_u16(buf, &pos, 0xBBCC), 0);
	CU_ASSERT_EQUAL(pos, 3);
	CU_ASSERT_EQUAL(rtp_write_u32(buf, &pos, 0xDDEEFF00), 0);
	CU_ASSERT_EQUAL(pos, 7);

	CU_ASSERT_EQUAL(rtp_read_u8(buf, &read_pos, &v8), 0);
	CU_ASSERT_EQUAL(v8, 0xAA);
	CU_ASSERT_EQUAL(read_pos, 1);
	CU_ASSERT_EQUAL(rtp_read_u16(buf, &read_pos, &v16), 0);
	CU_ASSERT_EQUAL(v16, 0xBBCC);
	CU_ASSERT_EQUAL(read_pos, 3);
	CU_ASSERT_EQUAL(rtp_read_u32(buf, &read_pos, &v32), 0);
	CU_ASSERT_EQUAL(v32, 0xDDEEFF00);
	CU_ASSERT_EQUAL(read_pos, 7);

	pomp_buffer_unref(buf);
}


CU_TestInfo s_test_priv[] = {
	{FN("diff_seqnum"), &test_priv_diff_seqnum},
	{FN("write_read_u8"), &test_priv_write_read_u8},
	{FN("write_read_u16_network_order"),
	 &test_priv_write_read_u16_network_order},
	{FN("write_read_u32_network_order"),
	 &test_priv_write_read_u32_network_order},
	{FN("write_read_u64_network_order"),
	 &test_priv_write_read_u64_network_order},
	{FN("read_insufficient_buffer"), &test_priv_read_insufficient_buffer},
	{FN("write_sequence_advances_pos"),
	 &test_priv_write_sequence_advances_pos},
	CU_TEST_INFO_NULL,
};
