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

#include "rtp_priv.h"

#define SKEW_WINDOW_SLIDING

#ifdef SKEW_WINDOW_SLIDING
#	define SKEW_WINDOW_MAX_SIZE 512
#	define SKEW_WINDOW_TIMEOUT 2000000
#	define SKEW_AVG_ALPHA 128
#else
#	define SKEW_WINDOW_MAX_SIZE 400
#	define SKEW_WINDOW_TIMEOUT 5000000
#	define SKEW_AVG_ALPHA 64
#endif

#define SKEW_LARGE_GAP 1000000

#define JITTER_AVG_ALPHA 16


struct rtp_jitter {
	struct rtp_jitter_cfg cfg;
	struct rtp_jitter_cbs cbs;
	void *userdata;

	struct list_node packets;
	uint16_t next_seqnum;

	uint64_t first_rx_timestamp;
	uint64_t first_rtp_timestamp;

	uint64_t last_rx_timestamp;
	uint64_t last_rtp_timestamp;

	int64_t window[SKEW_WINDOW_MAX_SIZE];
	uint32_t window_pos;
	uint32_t window_size;
	uint64_t window_start_timestamp;
	int64_t window_min;
	int64_t skew_avg;

	/* Estimated jitter (in us) */
	uint32_t jitter_avg;
};


static void reset_skew(struct rtp_jitter *self,
		       uint64_t rx_timestamp,
		       uint64_t rtp_timestamp)
{
	self->first_rx_timestamp = rx_timestamp;
	self->first_rtp_timestamp = rtp_timestamp;
	self->window_pos = 0;
	self->window_size = 0;
	self->window_start_timestamp = 0;
	self->window_min = 0;
	self->skew_avg = 0;
}


/**
 * Interarrival jitter computation
 * J(i) = J(i-1) + (|D(i-1,i)| - J(i-1))/16
 * D(i,j) = (Rj - Ri) - (Sj - Si) = (Rj - Sj) - (Ri - Si)
 * Si is the RTP timestamp from packet i
 * Ri is the time of arrival in RTP timestamp units for packet i
 */
static void compute_jitter(struct rtp_jitter *self,
			   uint64_t rx_timestamp,
			   uint64_t rtp_timestamp)
{
	uint32_t clk_rate = self->cfg.clk_rate;
	int64_t delta_rx = 0;
	int64_t delta_rtp = 0;
	int64_t jitter = 0;

	/* Compute delta in us */
	delta_rx = rx_timestamp - self->last_rx_timestamp;
	delta_rtp = rtp_timestamp - self->last_rtp_timestamp;
	if (delta_rtp > 0)
		delta_rtp = rtp_timestamp_to_us(delta_rtp, clk_rate);
	else
		delta_rtp = -rtp_timestamp_to_us(-delta_rtp, clk_rate);

	jitter = delta_rx - delta_rtp;
	if (jitter < 0)
		jitter = -jitter;
	self->jitter_avg += (jitter - self->jitter_avg) / JITTER_AVG_ALPHA;
}


static uint64_t compute_skew(struct rtp_jitter *self,
			     uint64_t rx_timestamp,
			     uint64_t rtp_timestamp)
{
	uint32_t clk_rate = self->cfg.clk_rate;
	int64_t delta_recv = 0;
	int64_t delta_send = 0;
	int64_t skew = 0;
	uint64_t out_timestamp = 0;

	/* Compute delta in us */
	delta_send = rtp_timestamp - self->first_rtp_timestamp;
	if (delta_send < 0) {
		/* The sender probably restarted */
		delta_send = -rtp_timestamp_to_us(-delta_send, clk_rate);
		ULOGD("reset skew: delta_send(%.6f) < 0",
		      delta_send / 1000000.0);
		reset_skew(self, rx_timestamp, rtp_timestamp);
		delta_send = 0;
	} else {
		delta_send = rtp_timestamp_to_us(delta_send, clk_rate);
	}
	delta_recv = rx_timestamp - self->first_rx_timestamp;

	/* Current skew */
	skew = delta_recv - delta_send;

	/* Check for large gaps */
	if (skew - self->skew_avg < -SKEW_LARGE_GAP ||
	    skew - self->skew_avg > SKEW_LARGE_GAP) {
		ULOGD("reset skew: skew(%.6f) - skew_avg(%.6f) too large",
		      skew / 1000000.0,
		      self->skew_avg / 1000000.0);
		reset_skew(self, rx_timestamp, rtp_timestamp);
		delta_send = 0;
		delta_recv = 0;
		skew = 0;
	}

#ifdef SKEW_WINDOW_SLIDING
	/* Are we at initialization stage? */
	if (self->window_size == 0) {
		/* Save value */
		self->window[self->window_pos] = skew;
		if (self->window_pos == 0) {
			/* First value in window */
			self->window_start_timestamp = rx_timestamp;
			self->window_min = skew;
		} else if (skew < self->window_min) {
			/* New minimum found */
			self->window_min = skew;
		}

		/* Are we done? */
		self->window_pos++;
		if (self->window_pos >= SKEW_WINDOW_MAX_SIZE ||
		    rx_timestamp >= self->window_start_timestamp +
					    SKEW_WINDOW_TIMEOUT) {
			self->window_size = self->window_pos;
			self->window_pos = 0;
			self->skew_avg = self->window_min;
		} else if (rx_timestamp >= self->window_start_timestamp) {
			uint32_t perc_time =
				(rx_timestamp - self->window_start_timestamp) *
				100 / SKEW_WINDOW_TIMEOUT;
			uint32_t perc_window =
				self->window_pos * 100 / SKEW_WINDOW_MAX_SIZE;
			uint32_t perc = perc_time > perc_window ? perc_time
								: perc_window;
			/* Parabolic function */
			perc = perc * perc;
			self->skew_avg += perc *
					  (self->window_min - self->skew_avg) /
					  10000;
		} else {
			/* Might be different links */
			ULOGD("reset skew: "
			      "window_start_timestamp > rx_timestamp");
			reset_skew(self, rx_timestamp, rtp_timestamp);
			out_timestamp = rx_timestamp;
			goto out;
		}
	} else {
		/* Remember the old value and set the new one */
		int64_t old = self->window[self->window_pos];
		self->window[self->window_pos] = skew;

		if (skew < self->window_min) {
			/* New minimum found */
			self->window_min = skew;
		} else if (old == self->window_min) {
			/* We replace the current min value, find the new min */
			self->window_min = INT64_MAX;
			for (uint32_t i = 0; i < self->window_size; i++) {
				if (self->window[i] == old) {
					/* We found the old min again, save it
					 * and break */
					self->window_min = self->window[i];
					break;
				} else if (self->window[i] < self->window_min) {
					/* Save new min, and continue search */
					self->window_min = self->window[i];
				}
			}
		}

		/* Update position and wrap if needed */
		self->window_pos++;
		if (self->window_pos >= self->window_size)
			self->window_pos = 0;

		/* Sliding average */
		self->skew_avg +=
			(self->window_min - self->skew_avg) / SKEW_AVG_ALPHA;
	}
#else
	/* Fill the window */
	if (self->window_size == 0)
		self->window_start_timestamp = rx_timestamp;
	self->window[self->window_size] = skew;
	self->window_size++;

	if ((self->window_size >= SKEW_WINDOW_MAX_SIZE) ||
	    (self->window_size >= SKEW_WINDOW_MAX_SIZE / 2 &&
	     rx_timestamp >=
		     self->window_start_timestamp + SKEW_WINDOW_TIMEOUT)) {
		/* Window is full or half-full and on timeout */
		self->window_min = self->window[0];
		for (uint32_t i = 1; i < self->window_size; i++) {
			if (self->window[i] < self->window_min)
				self->window_min = self->window[i];
		}

		/* Sliding average */
		self->skew_avg +=
			(self->window_min - self->skew_avg) / SKEW_AVG_ALPHA;

		/* Reset the window */
		self->window_size = 0;
	}
#endif

	/* Estimated out timestamp */
	out_timestamp = self->first_rx_timestamp + delta_send + self->skew_avg;

	/* Make sure we don't go backwards */
	if (out_timestamp + self->cfg.delay < rx_timestamp) {
		ULOGD("reset skew: out(%.6f) + delay(%.6f) < in(%.6f)",
		      out_timestamp / 1000000.0,
		      self->cfg.delay / 1000000.0,
		      rx_timestamp / 1000000.0);
		reset_skew(self, rx_timestamp, rtp_timestamp);
		out_timestamp = rx_timestamp;
	}

out:
	return out_timestamp;
}


int rtp_jitter_new(const struct rtp_jitter_cfg *cfg,
		   const struct rtp_jitter_cbs *cbs,
		   void *userdata,
		   struct rtp_jitter **ret_obj)
{
	struct rtp_jitter *self = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(cfg == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cfg->clk_rate == 0, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	*ret_obj = NULL;

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->cfg = *cfg;
	self->cbs = *cbs;
	self->userdata = userdata;
	list_init(&self->packets);

	*ret_obj = self;
	return 0;
}


int rtp_jitter_destroy(struct rtp_jitter *self)
{
	if (self == NULL)
		return 0;

	rtp_jitter_clear(self, 0);
	free(self);
	return 0;
}


int rtp_jitter_clear(struct rtp_jitter *self, uint16_t next_seqnum)
{
	struct rtp_pkt *pkt = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	/* Destroy all packets in the queue */
	while (!list_is_empty(&self->packets)) {
		pkt = list_entry(
			list_first(&self->packets), struct rtp_pkt, node);
		list_del(&pkt->node);
		rtp_pkt_destroy(pkt);
	}

	self->first_rx_timestamp = 0;
	self->first_rtp_timestamp = 0;
	self->last_rx_timestamp = 0;
	self->last_rtp_timestamp = 0;
	self->window_size = 0;
	self->window_start_timestamp = 0;
	self->skew_avg = 0;
	self->jitter_avg = 0;

	/* Set the seq num of the next expected packet */
	self->next_seqnum = next_seqnum;
	return 0;
}


int rtp_jitter_enqueue(struct rtp_jitter *self, struct rtp_pkt *pkt)
{
	uint64_t in_timestamp = 0;
	uint64_t rtp_timestamp = 0;
	struct rtp_pkt *item = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(pkt == NULL, EINVAL);

	in_timestamp = pkt->in_timestamp;
	rtp_timestamp = pkt->rtp_timestamp;

	if (self->first_rx_timestamp == 0 || self->first_rtp_timestamp == 0)
		reset_skew(self, in_timestamp, rtp_timestamp);

	if (self->last_rx_timestamp != 0 && self->last_rtp_timestamp != 0)
		compute_jitter(self, in_timestamp, rtp_timestamp);
	pkt->out_timestamp = compute_skew(self, in_timestamp, rtp_timestamp);

	self->last_rx_timestamp = in_timestamp;
	self->last_rtp_timestamp = rtp_timestamp;

	if (rtp_diff_seqnum(self->next_seqnum, pkt->header.seqnum) > 0) {
		/* Old packet */
		rtp_pkt_destroy(pkt);
		return 0;
	}

	list_walk_entry_backward(&self->packets, item, node)
	{
		int16_t diff = rtp_diff_seqnum(item->header.seqnum,
					       pkt->header.seqnum);
		if (diff < 0) {
			list_add_after(&item->node, &pkt->node);
			return 0;
		}
		if (item->header.seqnum == pkt->header.seqnum) {
			/* Duplicate packet */
			rtp_pkt_destroy(pkt);
			return 0;
		}
	}

	/* First packet (or empty list) */
	list_add_before(list_first(&self->packets), &pkt->node);
	return 0;
}


int rtp_jitter_process(struct rtp_jitter *self, uint64_t cur_timestamp)
{
	struct rtp_pkt *pkt = NULL;
	uint32_t gap = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	while (!list_is_empty(&self->packets)) {
		/* Get first packet */
		pkt = list_entry(
			list_first(&self->packets), struct rtp_pkt, node);

		/* Is it the next one to process? */
		if (pkt->header.seqnum == self->next_seqnum)
			goto do_process;

		/* Is it time to process it? */
		if (cur_timestamp >= pkt->out_timestamp + self->cfg.delay)
			goto do_process;

		/* No more packet eligible for process */
		break;

	/* codecheck_ignore[INDENTED_LABEL] */
	do_process:
		gap = rtp_diff_seqnum(pkt->header.seqnum, self->next_seqnum);
		(self->cbs.process_pkt)(self, pkt, gap, self->userdata);
		self->next_seqnum = (pkt->header.seqnum + 1) & 0xffff;
		list_del(&pkt->node);
		rtp_pkt_destroy(pkt);
	}

	return 0;
}


int rtp_jitter_get_info(struct rtp_jitter *self,
			uint32_t *clk_rate,
			uint32_t *jitter_avg,
			int64_t *skew_avg)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (clk_rate != NULL)
		*clk_rate = self->cfg.clk_rate;
	if (jitter_avg != NULL)
		*jitter_avg = self->jitter_avg;
	if (skew_avg != NULL)
		*skew_avg = self->skew_avg;

	return 0;
}
