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

#ifndef _RTP_H_
#define _RTP_H_

#include <errno.h>
#include <stdint.h>
#include <time.h>

#include <futils/list.h>
#include <libpomp.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** To be used for all public API */
#ifdef RTP_API_EXPORTS
#	ifdef _WIN32
#		define RTP_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define RTP_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !RTP_API_EXPORTS */
#	define RTP_API
#endif /* !RTP_API_EXPORTS */

#include "rtp/ntp.h"
#include "rtp/rtcp_pkt.h"
#include "rtp/rtp_jitter.h"
#include "rtp/rtp_pkt.h"


static inline uint64_t rtp_timestamp_to_us(uint64_t rtp_timestamp,
					   uint32_t clk_rate)
{
	if (clk_rate == 0)
		return 0;
	else
		return (rtp_timestamp * 1000000 + clk_rate / 2) / clk_rate;
}


static inline uint64_t rtp_timestamp_from_us(uint64_t us, uint32_t clk_rate)
{
	return (us * clk_rate + 500000) / 1000000;
}


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_RTP_H_ */
