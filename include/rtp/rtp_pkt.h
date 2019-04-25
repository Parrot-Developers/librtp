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

#ifndef _RTP_PKT_H_
#define _RTP_PKT_H_


#define RTP_PKT_VERSION 2

#define RTP_PKT_HEADER_SIZE 12

#define RTP_PKT_HEADER_FLAGS_VERSION_SHIFT 14
#define RTP_PKT_HEADER_FLAGS_VERSION_MASK 0x0003

#define RTP_PKT_HEADER_FLAGS_PADDING_SHIFT 13
#define RTP_PKT_HEADER_FLAGS_PADDING_MASK 0x0001

#define RTP_PKT_HEADER_FLAGS_EXTENSION_SHIFT 12
#define RTP_PKT_HEADER_FLAGS_EXTENSION_MASK 0x0001

#define RTP_PKT_HEADER_FLAGS_CSRC_SHIFT 8
#define RTP_PKT_HEADER_FLAGS_CSRC_MASK 0x000f

#define RTP_PKT_HEADER_FLAGS_MARKER_SHIFT 7
#define RTP_PKT_HEADER_FLAGS_MARKER_MASK 0x0001

#define RTP_PKT_HEADER_FLAGS_PAYLOAD_TYPE_SHIFT 0
#define RTP_PKT_HEADER_FLAGS_PAYLOAD_TYPE_MASK 0x007f

#define RTP_PKT_HEADER_FLAGS_GET(_flags, _name)                                \
	(((_flags) >> RTP_PKT_HEADER_FLAGS_##_name##_SHIFT) &                  \
	 RTP_PKT_HEADER_FLAGS_##_name##_MASK)

#define RTP_PKT_HEADER_FLAGS_SET(_flags, _name, _val)                          \
	((_flags) |= ((_val)&RTP_PKT_HEADER_FLAGS_##_name##_MASK)              \
		     << RTP_PKT_HEADER_FLAGS_##_name##_SHIFT)


/**
 * 5.1 RTP Fixed Header Fields
 *
 *   0                   1                   2                   3
 *   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |V=2|P|X|  CC   |M|     PT      |       sequence number         |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |                           timestamp                           |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *  |           synchronization source (SSRC) identifier            |
 *  +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 *  |            contributing source (CSRC) identifiers             |
 *  |                             ....                              |
 *  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 *  version (V): 2 bits
 *  padding (P): 1 bit
 *  extension (X): 1 bit
 *  CSRC count (CC): 4 bits
 *  marker (M): 1 bit
 *  payload type (PT): 7 bits
 *  sequence number: 16 bits
 *  timestamp: 32 bits
 *  SSRC: 32 bits
 *  CSRC list: 0 to 15 items, 32 bits each
 *
 */
struct rtp_pkt_header {
	uint16_t flags;
	uint16_t seqnum;
	uint32_t timestamp;
	uint32_t ssrc;
};


struct rtp_pkt {
	struct rtp_pkt_header header;

	struct {
		struct pomp_buffer *buf;
		const uint8_t *cdata;
		size_t len;
	} raw;

	struct {
		uint16_t id;
		size_t off;
		size_t len;
	} extheader;

	struct {
		size_t off;
		size_t len;
	} payload;

	/* To be used in a list */
	struct list_node node;

	/* Input timestamp (in us from monotonic clock) */
	uint64_t in_timestamp;

	/* Output timestamp (in us from monotonic clock, unskewed) */
	uint64_t out_timestamp;

	/* Extended RTP timestamp (without wrap) */
	uint64_t rtp_timestamp;

	/** Priority of packet (inherited from the highest priority
	 *  of included NALUs (sender only) (low numbers have more priority) */
	uint32_t priority;

	/** Importance of packet (inherited from the highest importance
	 *  of included NALUs (sender only) (low numbers are more important) */
	uint32_t importance;
};


RTP_API
int rtp_pkt_new(struct rtp_pkt **ret_obj);


RTP_API
int rtp_pkt_clone(const struct rtp_pkt *pkt, struct rtp_pkt **ret_obj);


RTP_API
int rtp_pkt_destroy(struct rtp_pkt *pkt);


RTP_API
int rtp_pkt_finalize_header(struct rtp_pkt *pkt);


RTP_API
int rtp_pkt_read(struct pomp_buffer *buf, struct rtp_pkt *pkt);


#endif /* !_RTP_PKT_H_ */
