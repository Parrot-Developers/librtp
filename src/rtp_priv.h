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

#ifndef _RTP_PRIV_H_
#define _RTP_PRIV_H_

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#define ULOG_TAG rtp
#include <ulog.h>

#include <futils/list.h>
#include <libpomp.h>

#include "rtp/rtp.h"


static inline int rtp_write_u8(struct pomp_buffer *buf, size_t *pos, uint8_t v)
{
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
rtp_write_u16(struct pomp_buffer *buf, size_t *pos, uint16_t v)
{
	v = htons(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
rtp_write_u32(struct pomp_buffer *buf, size_t *pos, uint32_t v)
{
	v = htonl(v);
	return pomp_buffer_write(buf, pos, &v, sizeof(v));
}


static inline int
rtp_write_u64(struct pomp_buffer *buf, size_t *pos, uint64_t v)
{
	uint32_t _v[2];
	_v[0] = htonl((v >> 32) & 0xffffffff);
	_v[1] = htonl(v & 0xffffffff);
	return pomp_buffer_write(buf, pos, _v, sizeof(_v));
}


static inline int rtp_read_u8(struct pomp_buffer *buf, size_t *pos, uint8_t *v)
{
	return pomp_buffer_read(buf, pos, v, sizeof(*v));
}


static inline int
rtp_read_u16(struct pomp_buffer *buf, size_t *pos, uint16_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohs(*v);
	return res;
}


static inline int
rtp_read_u32(struct pomp_buffer *buf, size_t *pos, uint32_t *v)
{
	int res = 0;
	res = pomp_buffer_read(buf, pos, v, sizeof(*v));
	if (res == 0)
		*v = ntohl(*v);
	return res;
}


static inline int
rtp_read_u64(struct pomp_buffer *buf, size_t *pos, uint64_t *v)
{
	int res = 0;
	uint32_t _v[2];
	res = pomp_buffer_read(buf, pos, _v, sizeof(_v));
	if (res == 0)
		*v = ((uint64_t)ntohl(_v[0]) << 32) | ntohl(_v[1]);
	return res;
}


static inline int16_t rtp_diff_seqnum(uint16_t sq1, uint16_t sq2)
{
	return (int16_t)(sq1 - sq2);
}


#endif /* !_RTP_PRIV_H_ */
