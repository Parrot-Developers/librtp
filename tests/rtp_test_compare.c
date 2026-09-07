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
#include "rtp/rtp_pkt.h"


struct pomp_buffer *make_pomp_buffer_from_bytes(const uint8_t *data, size_t len)
{
	struct pomp_buffer *buf = pomp_buffer_new_with_data(data, len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	return buf;
}


struct pomp_buffer *make_pomp_buffer(size_t capacity)
{
	struct pomp_buffer *buf = pomp_buffer_new(capacity);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	return buf;
}


void fill_rtp_pkt_header(struct rtp_pkt_header *hdr,
			 uint8_t version,
			 uint8_t padding,
			 uint8_t extension,
			 uint8_t csrc,
			 uint8_t marker,
			 uint8_t pt,
			 uint16_t seqnum,
			 uint32_t timestamp,
			 uint32_t ssrc)
{
	uint16_t flags = 0;

	RTP_PKT_HEADER_FLAGS_SET(flags, VERSION, version);
	RTP_PKT_HEADER_FLAGS_SET(flags, PADDING, padding);
	RTP_PKT_HEADER_FLAGS_SET(flags, EXTENSION, extension);
	RTP_PKT_HEADER_FLAGS_SET(flags, CSRC, csrc);
	RTP_PKT_HEADER_FLAGS_SET(flags, MARKER, marker);
	RTP_PKT_HEADER_FLAGS_SET(flags, PAYLOAD_TYPE, pt);

	hdr->flags = flags;
	hdr->seqnum = seqnum;
	hdr->timestamp = timestamp;
	hdr->ssrc = ssrc;
}


void compare_rtp_pkt_header(const struct rtp_pkt_header *h1,
			    const struct rtp_pkt_header *h2)
{
	CU_ASSERT_EQUAL(h1->flags, h2->flags);
	CU_ASSERT_EQUAL(h1->seqnum, h2->seqnum);
	CU_ASSERT_EQUAL(h1->timestamp, h2->timestamp);
	CU_ASSERT_EQUAL(h1->ssrc, h2->ssrc);
}


void compare_rtcp_pkt_report_block(const struct rtcp_pkt_report_block *b1,
				   const struct rtcp_pkt_report_block *b2)
{
	CU_ASSERT_EQUAL(b1->ssrc, b2->ssrc);
	CU_ASSERT_EQUAL(b1->fraction, b2->fraction);
	CU_ASSERT_EQUAL(b1->lost, b2->lost);
	CU_ASSERT_EQUAL(b1->ext_highest_seqnum, b2->ext_highest_seqnum);
	CU_ASSERT_EQUAL(b1->jitter, b2->jitter);
	CU_ASSERT_EQUAL(b1->lsr.seconds, b2->lsr.seconds);
	CU_ASSERT_EQUAL(b1->lsr.fraction, b2->lsr.fraction);
	CU_ASSERT_EQUAL(b1->dlsr, b2->dlsr);
}


void compare_rtcp_pkt_sender_report(const struct rtcp_pkt_sender_report *s1,
				    const struct rtcp_pkt_sender_report *s2)
{
	CU_ASSERT_EQUAL(s1->ssrc, s2->ssrc);
	CU_ASSERT_EQUAL(s1->ntp_timestamp.seconds, s2->ntp_timestamp.seconds);
	CU_ASSERT_EQUAL(s1->ntp_timestamp.fraction, s2->ntp_timestamp.fraction);
	CU_ASSERT_EQUAL(s1->rtp_timestamp, s2->rtp_timestamp);
	CU_ASSERT_EQUAL(s1->sender_packet_count, s2->sender_packet_count);
	CU_ASSERT_EQUAL(s1->sender_byte_count, s2->sender_byte_count);
	CU_ASSERT_EQUAL(s1->report_count, s2->report_count);

	if (s1->report_count != s2->report_count)
		return;
	for (uint32_t i = 0; i < s1->report_count; i++)
		compare_rtcp_pkt_report_block(&s1->reports[i], &s2->reports[i]);
}


void compare_rtcp_pkt_receiver_report(const struct rtcp_pkt_receiver_report *r1,
				      const struct rtcp_pkt_receiver_report *r2)
{
	CU_ASSERT_EQUAL(r1->ssrc, r2->ssrc);
	CU_ASSERT_EQUAL(r1->report_count, r2->report_count);

	if (r1->report_count != r2->report_count)
		return;
	for (uint32_t i = 0; i < r1->report_count; i++)
		compare_rtcp_pkt_report_block(&r1->reports[i], &r2->reports[i]);
}


void compare_rtcp_pkt_bye(const struct rtcp_pkt_bye *b1,
			  const struct rtcp_pkt_bye *b2)
{
	CU_ASSERT_EQUAL(b1->source_count, b2->source_count);
	if (b1->source_count == b2->source_count) {
		for (uint32_t i = 0; i < b1->source_count; i++)
			CU_ASSERT_EQUAL(b1->sources[i], b2->sources[i]);
	}

	CU_ASSERT_EQUAL(b1->reason_len, b2->reason_len);
	if (b1->reason_len == b2->reason_len && b1->reason_len > 0 &&
	    b1->reason != NULL && b2->reason != NULL) {
		CU_ASSERT_EQUAL(memcmp(b1->reason, b2->reason, b1->reason_len),
				0);
	}
}


void compare_rtcp_pkt_app(const struct rtcp_pkt_app *a1,
			  const struct rtcp_pkt_app *a2)
{
	CU_ASSERT_EQUAL(a1->ssrc, a2->ssrc);
	CU_ASSERT_EQUAL(a1->name, a2->name);
	CU_ASSERT_EQUAL(a1->subtype, a2->subtype);
	CU_ASSERT_EQUAL(a1->data_len, a2->data_len);
	if (a1->data_len == a2->data_len && a1->data_len > 0 &&
	    a1->data != NULL && a2->data != NULL) {
		CU_ASSERT_EQUAL(memcmp(a1->data, a2->data, a1->data_len), 0);
	}
}


void compare_rtcp_pkt_rtpfb_report(const struct rtcp_pkt_rtpfb_report *r1,
				   const struct rtcp_pkt_rtpfb_report *r2)
{
	CU_ASSERT_EQUAL(r1->sender_ssrc, r2->sender_ssrc);
	CU_ASSERT_EQUAL(r1->media_ssrc, r2->media_ssrc);
	CU_ASSERT_EQUAL(r1->base_seq, r2->base_seq);
	CU_ASSERT_EQUAL(r1->status_count, r2->status_count);
	CU_ASSERT_EQUAL(r1->ref_time, r2->ref_time);
	CU_ASSERT_EQUAL(r1->feedback_pkt_count, r2->feedback_pkt_count);

	if (r1->status_count != r2->status_count)
		return;
	for (uint16_t i = 0; i < r1->status_count; i++) {
		CU_ASSERT_EQUAL(r1->feedbacks[i].seq_num,
				r2->feedbacks[i].seq_num);
		CU_ASSERT_EQUAL(r1->feedbacks[i].pkt_status_symbol,
				r2->feedbacks[i].pkt_status_symbol);
		CU_ASSERT_EQUAL(r1->feedbacks[i].recv_delta,
				r2->feedbacks[i].recv_delta);
	}
}
