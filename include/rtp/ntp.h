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

#ifndef _NTP_H_
#define _NTP_H_


struct ntp_timestamp64 {
	uint32_t seconds;
	uint32_t fraction;
};


struct ntp_timestamp32 {
	uint16_t seconds;
	uint16_t fraction;
};


static inline int ntp_timestamp64_diff_us(const struct ntp_timestamp64 *t1,
					  const struct ntp_timestamp64 *t2,
					  int64_t *d)
{
	if (t1 == NULL || t2 == NULL || d == NULL)
		return -EINVAL;
	*d = ((int64_t)t1->seconds - (int64_t)t2->seconds) * 1000000;
	*d += (((int64_t)t1->fraction - (int64_t)t2->fraction) * 1000000) >> 32;
	return 0;
}


static inline int ntp_timestamp64_to_us(const struct ntp_timestamp64 *t,
					uint64_t *us)
{
	if (t == NULL || us == NULL)
		return -EINVAL;
	*us = ((uint64_t)t->seconds * 1000000) +
	      (((uint64_t)t->fraction * 1000000) >> 32);
	return 0;
}


static inline int ntp_timestamp64_from_us(struct ntp_timestamp64 *t,
					  uint64_t us)
{
	if (t == NULL)
		return -EINVAL;
	t->seconds = us / 1000000;
	t->fraction = ((uint64_t)(us % 1000000) << 32) / 1000000;
	return 0;
}


static inline int ntp_timestamp64_to_timespec(const struct ntp_timestamp64 *t,
					      struct timespec *ts)
{
	if (t == NULL || ts == NULL)
		return -EINVAL;
	ts->tv_sec = t->seconds;
	ts->tv_nsec = ((uint64_t)t->fraction * 1000000000) >> 32;
	return 0;
}


static inline int ntp_timestamp64_from_timespec(struct ntp_timestamp64 *t,
						const struct timespec *ts)
{
	if (t == NULL || ts == NULL)
		return -EINVAL;
	t->seconds = ts->tv_sec;
	t->fraction = ((uint64_t)ts->tv_nsec << 32) / 1000000000;
	return 0;
}


static inline int
ntp_timestamp64_to_ntp_timestamp32(const struct ntp_timestamp64 *t1,
				   struct ntp_timestamp32 *t2)
{
	if (t1 == NULL || t2 == NULL)
		return -EINVAL;
	t2->seconds = t1->seconds & 0xffff;
	t2->fraction = (t1->fraction >> 16) & 0xffff;
	return 0;
}


static inline int ntp_timestamp32_diff_us(const struct ntp_timestamp32 *t1,
					  const struct ntp_timestamp32 *t2,
					  int64_t *d)
{
	if (t1 == NULL || t2 == NULL || d == NULL)
		return -EINVAL;
	*d = ((int64_t)t1->seconds - (int64_t)t2->seconds) * 1000000;
	*d += (((int64_t)t1->fraction - (int64_t)t2->fraction) * 1000000) >> 16;
	return 0;
}


static inline int ntp_timestamp32_to_us(const struct ntp_timestamp32 *t,
					uint64_t *us)
{
	if (t == NULL || us == NULL)
		return -EINVAL;
	*us = ((uint64_t)t->seconds * 1000000) +
	      (((uint64_t)t->fraction * 1000000) >> 16);
	return 0;
}


static inline int ntp_timestamp32_from_us(struct ntp_timestamp32 *t,
					  uint64_t us)
{
	if (t == NULL)
		return -EINVAL;
	t->seconds = us / 1000000;
	t->fraction = ((uint64_t)(us % 1000000) << 16) / 1000000;
	return 0;
}


static inline int ntp_timestamp32_to_timespec(const struct ntp_timestamp32 *t,
					      struct timespec *ts)
{
	if (t == NULL || ts == NULL)
		return -EINVAL;
	ts->tv_sec = t->seconds;
	ts->tv_nsec = ((uint64_t)t->fraction * 1000000000) >> 16;
	return 0;
}


static inline int ntp_timestamp32_from_timespec(struct ntp_timestamp32 *t,
						const struct timespec *ts)
{
	if (t == NULL || ts == NULL)
		return -EINVAL;
	t->seconds = ts->tv_sec;
	t->fraction = ((uint64_t)ts->tv_nsec << 16) / 1000000000;
	return 0;
}


static inline int
ntp_timestamp32_to_ntp_timestamp64(const struct ntp_timestamp32 *t1,
				   struct ntp_timestamp64 *t2)
{
	if (t1 == NULL || t2 == NULL)
		return -EINVAL;
	t2->seconds = t1->seconds;
	t2->fraction = t1->fraction << 16;
	return 0;
}


#endif /* !_NTP_H_ */
