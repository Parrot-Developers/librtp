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

#ifndef _RTP_TEST_H_
#define _RTP_TEST_H_

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <futils/list.h>
#include <libpomp.h>

#include "rtp/rtp.h"

#define FN(_name) (char *)_name

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


/* Per-area suites, one CU_TestInfo[] array per rtp_test_<area>.c file */
extern CU_TestInfo s_test_ntp[];
extern CU_TestInfo s_test_priv[];
extern CU_TestInfo s_test_timestamp[];
extern CU_TestInfo s_test_rtp_pkt[];
extern CU_TestInfo s_test_rtcp_pkt[];
extern CU_TestInfo s_test_rtp_jitter[];

/* Monkey/fuzz suite, only registered when 'monkey' is given on argv */
extern CU_TestInfo s_monkey_rtp[];


/*
 * rtp_test_compare.c: shared fixture builders and deep-compare helpers.
 */

struct pomp_buffer *make_pomp_buffer_from_bytes(const uint8_t *data,
						size_t len);

struct pomp_buffer *make_pomp_buffer(size_t capacity);

void fill_rtp_pkt_header(struct rtp_pkt_header *hdr,
			 uint8_t version,
			 uint8_t padding,
			 uint8_t extension,
			 uint8_t csrc,
			 uint8_t marker,
			 uint8_t pt,
			 uint16_t seqnum,
			 uint32_t timestamp,
			 uint32_t ssrc);

void compare_rtp_pkt_header(const struct rtp_pkt_header *h1,
			    const struct rtp_pkt_header *h2);

void compare_rtcp_pkt_report_block(const struct rtcp_pkt_report_block *b1,
				   const struct rtcp_pkt_report_block *b2);

void compare_rtcp_pkt_sender_report(const struct rtcp_pkt_sender_report *s1,
				    const struct rtcp_pkt_sender_report *s2);

void compare_rtcp_pkt_receiver_report(
	const struct rtcp_pkt_receiver_report *r1,
	const struct rtcp_pkt_receiver_report *r2);

void compare_rtcp_pkt_bye(const struct rtcp_pkt_bye *b1,
			  const struct rtcp_pkt_bye *b2);

void compare_rtcp_pkt_app(const struct rtcp_pkt_app *a1,
			  const struct rtcp_pkt_app *a2);

void compare_rtcp_pkt_rtpfb_report(const struct rtcp_pkt_rtpfb_report *r1,
				   const struct rtcp_pkt_rtpfb_report *r2);


#endif /* !_RTP_TEST_H_ */
