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

#ifndef _RTCP_PKT_H_
#define _RTCP_PKT_H_


#define RTCP_PKT_VERSION 2

#define RTCP_PKT_HEADER_SIZE 4

#define RTCP_PKT_HEADER_FLAGS_VERSION_SHIFT 6
#define RTCP_PKT_HEADER_FLAGS_VERSION_MASK 0x03

#define RTCP_PKT_HEADER_FLAGS_PADDING_SHIFT 5
#define RTCP_PKT_HEADER_FLAGS_PADDING_MASK 0x01

#define RTCP_PKT_HEADER_FLAGS_COUNT_SHIFT 0
#define RTCP_PKT_HEADER_FLAGS_COUNT_MASK 0x1f

#define RTCP_PKT_HEADER_FLAGS(_name, _flags)                                   \
	(((_flags) >> RTCP_PKT_HEADER_FLAGS_##_name##_SHIFT) &                 \
	 RTCP_PKT_HEADER_FLAGS_##_name##_MASK)

#define RTCP_PKT_TYPE_SR 200
#define RTCP_PKT_TYPE_RR 201
#define RTCP_PKT_TYPE_SDES 202
#define RTCP_PKT_TYPE_BYE 203
#define RTCP_PKT_TYPE_APP 204

#define RTCP_PKT_SDES_TYPE_END 0
#define RTCP_PKT_SDES_TYPE_CNAME 1
#define RTCP_PKT_SDES_TYPE_NAME 2
#define RTCP_PKT_SDES_TYPE_EMAIL 3
#define RTCP_PKT_SDES_TYPE_PHONE 4
#define RTCP_PKT_SDES_TYPE_LOC 5
#define RTCP_PKT_SDES_TYPE_TOOL 6
#define RTCP_PKT_SDES_TYPE_NOTE 7
#define RTCP_PKT_SDES_TYPE_PRIV 8


/**
 *         0                   1                   2                   3
 *         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * header |V=2|P|    ??   |      PT       |             length            |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * version (V): 2 bits
 * padding (P): 1 bit
 * packet type (PT): 8 bits
 * length: 16 bits
 */
struct rtcp_pkt_header {
	uint8_t flags;
	uint8_t type;
	uint16_t len;
};


/**
 *         0                   1                   2                   3
 *         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * report |                 SSRC_1 (SSRC of first source)                 |
 * block  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        | fraction lost |       cumulative number of packets lost       |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |           extended highest sequence number received           |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                      interarrival jitter                      |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                         last SR (LSR)                         |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                   delay since last SR (DLSR)                  |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * SSRC_n (source identifier): 32 bits
 * fraction lost: 8 bits
 * cumulative number of packets lost: 24 bits
 * extended highest sequence number received: 32 bits
 * interarrival jitter: 32 bits
 * last SR timestamp (LSR): 32 bits
 * delay since last SR (DLSR): 32 bits
 */
/* clang-format off */
struct rtcp_pkt_report_block {
	uint32_t ssrc;
	uint32_t fraction:8;
	int32_t lost:24;
	uint32_t ext_highest_seqnum;
	uint32_t jitter;
	struct ntp_timestamp32 lsr;
	uint32_t dlsr;
};
/* clang-format on */


/**
 * 6.4.1 SR: Sender Report RTCP Packet
 *
 *         0                   1                   2                   3
 *         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * header |V=2|P|    RC   |   PT=SR=200   |             length            |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                         SSRC of sender                        |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * sender |              NTP timestamp, most significant word             |
 * info   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |             NTP timestamp, least significant word             |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                         RTP timestamp                         |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                     sender's packet count                     |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                      sender's octet count                     |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * report |                 SSRC_1 (SSRC of first source)                 |
 * block  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   1    | fraction lost |       cumulative number of packets lost       |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |           extended highest sequence number received           |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                      interarrival jitter                      |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                         last SR (LSR)                         |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                   delay since last SR (DLSR)                  |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * report |                 SSRC_2 (SSRC of second source)                |
 * block  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   2    :                               ...                             :
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 *        |                  profile-specific extensions                  |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * reception report count (RC): 5 bits
 * SSRC: 32 bits
 * NTP timestamp: 64 bits
 * RTP timestamp: 32 bits
 * sender's packet count: 32 bits
 * sender's octet count: 32 bits
 */
struct rtcp_pkt_sender_report {
	uint32_t ssrc;
	struct ntp_timestamp64 ntp_timestamp;
	uint32_t rtp_timestamp;
	uint32_t sender_packet_count;
	uint32_t sender_byte_count;

	uint32_t report_count;
	struct rtcp_pkt_report_block reports[31];
};


/**
 * 6.4.2 RR: Receiver Report RTCP Packet
 *
 *         0                   1                   2                   3
 *         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * header |V=2|P|    RC   |   PT=RR=201   |             length            |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                     SSRC of packet sender                     |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * report |                 SSRC_1 (SSRC of first source)                 |
 * block  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   1    | fraction lost |       cumulative number of packets lost       |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |           extended highest sequence number received           |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                      interarrival jitter                      |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                         last SR (LSR)                         |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                   delay since last SR (DLSR)                  |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * report |                 SSRC_2 (SSRC of second source)                |
 * block  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   2    :                               ...                             :
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 *        |                  profile-specific extensions                  |
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * reception report count (RC): 5 bits
 * SSRC: 32 bits
 */
struct rtcp_pkt_receiver_report {
	uint32_t ssrc;

	uint32_t report_count;
	struct rtcp_pkt_report_block reports[31];
};


struct rtcp_pkt_sdes_item {
	uint8_t type;
	uint8_t data_len;
	const uint8_t *data;

	/* When type is RTCP_PKT_SDES_TYPE_PRIV */
	struct {
		uint8_t prefix_len;
		const uint8_t *prefix;
		uint8_t value_len;
		const uint8_t *value;
	} priv;
};


struct rtcp_pkt_sdes_chunk {
	uint32_t ssrc;

	uint32_t item_count;
	const struct rtcp_pkt_sdes_item *items;
};


/**
 * 6.5 SDES: Source Description RTCP Packet
 *
 *         0                   1                   2                   3
 *         0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *        +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * header |V=2|P|    SC   |  PT=SDES=202  |             length            |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * chunk  |                          SSRC/CSRC_1                          |
 *   1    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                           SDES items                          |
 *        |                              ...                              |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * chunk  |                          SSRC/CSRC_2                          |
 *   2    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *        |                           SDES items                          |
 *        |                              ...                              |
 *        +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 *
 * source count (SC): 5 bits
 */
struct rtcp_pkt_sdes {
	uint32_t chunk_count;
	const struct rtcp_pkt_sdes_chunk *chunks;
};


/**
 * 6.6 BYE: Goodbye RTCP Packet
 *
 *        0                   1                   2                   3
 *        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *       |V=2|P|    SC   |   PT=BYE=203  |             length            |
 *       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *       |                           SSRC/CSRC                           |
 *       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *       :                              ...                              :
 *       +=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+
 * (opt) |     length    |               reason for leaving            ...
 *       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * source count (SC): 5 bits
 */
struct rtcp_pkt_bye {
	uint32_t source_count;
	uint32_t sources[31];

	uint8_t reason_len;
	const uint8_t *reason;
};


/**
 * 6.7 APP: Application-Defined RTCP Packet
 *
 *     0                   1                   2                   3
 *     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |V=2|P| subtype |   PT=APP=204  |             length            |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                           SSRC/CSRC                           |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                          name (ASCII)                         |
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *    |                   application-dependent data                ...
 *    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 * subtype: 5 bits
 * name: 4 octets
 * application-dependent data: variable length
 */
struct rtcp_pkt_app {
	uint32_t ssrc;
	uint32_t name;

	uint8_t subtype;
	uint32_t data_len;
	const uint8_t *data;
};


struct rtcp_pkt_read_cbs {
	void (*sender_report)(const struct rtcp_pkt_sender_report *sr,
			      void *userdata);

	void (*receiver_report)(const struct rtcp_pkt_receiver_report *rr,
				void *userdata);

	void (*sdes_item)(uint32_t ssrc,
			  const struct rtcp_pkt_sdes_item *item,
			  void *userdata);

	void (*bye)(const struct rtcp_pkt_bye *bye, void *userdata);

	void (*app)(const struct rtcp_pkt_app *app, void *userdata);
};


RTP_API
const char *rtcp_pkt_type_str(uint8_t type);


RTP_API
const char *rtcp_pkt_sdes_type_str(uint8_t type);


RTP_API
int rtcp_pkt_write_sender_report(struct pomp_buffer *buf,
				 size_t *pos,
				 const struct rtcp_pkt_sender_report *sr);


RTP_API
int rtcp_pkt_write_receiver_report(struct pomp_buffer *buf,
				   size_t *pos,
				   const struct rtcp_pkt_receiver_report *rr);


RTP_API
int rtcp_pkt_write_sdes(struct pomp_buffer *buf,
			size_t *pos,
			const struct rtcp_pkt_sdes *sdes);


RTP_API
int rtcp_pkt_write_bye(struct pomp_buffer *buf,
		       size_t *pos,
		       const struct rtcp_pkt_bye *bye);


RTP_API
int rtcp_pkt_write_app(struct pomp_buffer *buf,
		       size_t *pos,
		       const struct rtcp_pkt_app *app);


RTP_API
int rtcp_pkt_read(struct pomp_buffer *buf,
		  const struct rtcp_pkt_read_cbs *cbs,
		  void *userdata);


#endif /* !_RTCP_PKT_H_ */
