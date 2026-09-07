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


static CU_SuiteInfo s_suites[] = {
	{FN("ntp"), NULL, NULL, s_test_ntp},
	{FN("priv"), NULL, NULL, s_test_priv},
	{FN("timestamp"), NULL, NULL, s_test_timestamp},
	{FN("rtp_pkt"), NULL, NULL, s_test_rtp_pkt},
	{FN("rtcp_pkt"), NULL, NULL, s_test_rtcp_pkt},
	{FN("rtp_jitter"), NULL, NULL, s_test_rtp_jitter},

	CU_SUITE_INFO_NULL,
};


/* Opt-in fuzz/"monkey" suite, only registered when 'monkey' is given on
 * argv (see main() below), mirroring libvideo-metadata's tst-vmeta. */
static CU_SuiteInfo s_monkey_suites[] = {
	{FN("monkey"), NULL, NULL, s_monkey_rtp},

	CU_SUITE_INFO_NULL,
};


static void run_automated(void)
{
	CU_automated_run_tests();
	CU_list_tests_to_file();
}


static void run_basic(void)
{
	CU_basic_set_mode(CU_BRM_VERBOSE);
	CU_basic_run_tests();
}


int main(int argc, char *argv[])
{
	const char *filename;
	bool monkey = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "monkey") == 0)
			monkey = true;
	}

	CU_initialize_registry();
	CU_register_suites(s_suites);
	if (monkey)
		CU_register_suites(s_monkey_suites);

	/* Set filename */
	filename = getenv("CUNIT_OUT_NAME");
	CU_set_output_filename(filename);

	/* Run tests */
	if (getenv("CUNIT_AUTOMATED") != NULL)
		run_automated();
	else
		run_basic();

	CU_cleanup_registry();

	return 0;
}
