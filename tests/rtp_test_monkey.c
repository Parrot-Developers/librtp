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

/*
 * Opt-in fuzz/"monkey" suite: feeds random buffers into the untrusted-wire-
 * data parsers to check for crashes and ref leaks across a much wider input
 * space than the hand-picked error-path cases in rtp_test_rtp_pkt.c /
 * rtp_test_rtcp_pkt.c. Only registered when 'monkey' is given on argv (see
 * rtp_test.c), mirroring packages/libvideo-metadata/tests's convention.
 */

#include "rtp_test.h"

#include "rtp/rtcp_pkt.h"
#include "rtp/rtp_pkt.h"

#include <futils/random.h>


#define MONKEY_ITERATIONS 500
#define MONKEY_MAX_LEN 256


static void test_monkey_rtp_pkt_read(void)
{
	uint8_t data[MONKEY_MAX_LEN];

	for (int i = 0; i < MONKEY_ITERATIONS; i++) {
		uint32_t len = futils_randomr32_maximum(MONKEY_MAX_LEN);
		struct pomp_buffer *buf = NULL;
		struct rtp_pkt *pkt = NULL;

		futils_random(data, len);
		buf = make_pomp_buffer_from_bytes(data, len);

		CU_ASSERT_EQUAL(rtp_pkt_new(&pkt), 0);
		/* Return value intentionally not checked: any value is a
		 * legal outcome on random input, we only care that it never
		 * crashes and that the packet can always be safely
		 * destroyed afterward (no ref leak on the error path). */
		rtp_pkt_read(buf, pkt);
		CU_ASSERT_EQUAL(rtp_pkt_destroy(pkt), 0);

		pomp_buffer_unref(buf);
	}
}


static void test_monkey_rtcp_pkt_read(void)
{
	uint8_t data[MONKEY_MAX_LEN];
	/* All-NULL callbacks are valid: every field is individually
	 * checked for NULL before being invoked. This suite is about
	 * crash-safety, not correctness of any decoded content. */
	struct rtcp_pkt_read_cbs cbs;
	memset(&cbs, 0, sizeof(cbs));

	for (int i = 0; i < MONKEY_ITERATIONS; i++) {
		uint32_t len = futils_randomr32_maximum(MONKEY_MAX_LEN);
		struct pomp_buffer *buf = NULL;

		futils_random(data, len);
		buf = make_pomp_buffer_from_bytes(data, len);

		rtcp_pkt_read(buf, &cbs, NULL);

		pomp_buffer_unref(buf);
	}
}


CU_TestInfo s_monkey_rtp[] = {
	{FN("rtp_pkt_read"), &test_monkey_rtp_pkt_read},
	{FN("rtcp_pkt_read"), &test_monkey_rtcp_pkt_read},
	CU_TEST_INFO_NULL,
};
