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

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rtp/rtp.h"


static void test_ntp_timestamp64(void)
{
	int64_t r = 0;
	static const struct {
		struct ntp_timestamp64 t1;
		struct ntp_timestamp64 t2;
		int64_t r;
	} table[] = {
		{{1, 0x40000000}, {1, 0x20000000}, 125000},
		{{1, 0x20000000}, {1, 0x40000000}, -125000},
		{{5, 0x40000000}, {2, 0x20000000}, 3125000},
		{{5, 0x20000000}, {2, 0x40000000}, 2875000},
		{{2, 0x40000000}, {5, 0x20000000}, -2875000},
		{{2, 0x20000000}, {5, 0x40000000}, -3125000},
	};

	for (uint32_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		ntp_timestamp64_diff_us(&table[i].t1, &table[i].t2, &r);
		printf("%" PRIi64 " %" PRIi64 "\n", r, table[i].r);
	}
}


static void test_ntp_timestamp32(void)
{
	int64_t r = 0;
	static const struct {
		struct ntp_timestamp32 t1;
		struct ntp_timestamp32 t2;
		int64_t r;
	} table[] = {
		{{1, 0x4000}, {1, 0x2000}, 125000},
		{{1, 0x2000}, {1, 0x4000}, -125000},
		{{5, 0x4000}, {2, 0x2000}, 3125000},
		{{5, 0x2000}, {2, 0x4000}, 2875000},
		{{2, 0x4000}, {5, 0x2000}, -2875000},
		{{2, 0x2000}, {5, 0x4000}, -3125000},
	};

	for (uint32_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
		ntp_timestamp32_diff_us(&table[i].t1, &table[i].t2, &r);
		printf("%ld %ld\n", r, table[i].r);
	}
}


int main()
{
	test_ntp_timestamp64();
	test_ntp_timestamp32();
	return 0;
}
