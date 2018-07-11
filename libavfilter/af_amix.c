/*
 * Audio Mix Filter
 * Copyright (c) 2012 Justin Ruggles <justin.ruggles@gmail.com>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Audio Mix Filter
 *
 * Mixes audio from multiple sources into a single output. The channel layout,
 * sample rate, and sample format will be the same for all inputs and the
 * output.
 */

#include "libavutil/attributes.h"
#include "libavutil/audio_fifo.h"
#include "libavutil/avassert.h"
#include "libavutil/avstring.h"
#include "libavutil/channel_layout.h"
#include "libavutil/common.h"
#include "libavutil/float_dsp.h"
#include "libavutil/mathematics.h"
#include "libavutil/opt.h"
#include "libavutil/samplefmt.h"

#include "audio.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"

#define INPUT_OFF      0    /**< input has reached EOF */
#define INPUT_ON       1    /**< input is active */
#define INPUT_INACTIVE 2    /**< input is on, but is currently inactive */

#define DURATION_LONGEST  0
#define DURATION_SHORTEST 1
#define DURATION_FIRST    2


#define MAX_CACHE_SECONDS 20 

#define CONSUME_CACHE_MILLISECONDS 500LL
#define MIN_CACHE_MILLISECONDS CONSUME_CACHE_MILLISECONDS * 2


typedef struct FrameInfo {
    int nb_samples;
    int64_t pts;
	int64_t length_pts;
	int64_t offset;
    AVFrame *frame;
    struct FrameInfo *next;
} FrameInfo;

/**
 * Linked list used to store timestamps and frame sizes of all frames in the
 * FIFO for the first input.
 *
 * This is needed to keep timestamps synchronized for the case where multiple
 * input frames are pushed to the filter for processing before a frame is
 * requested by the output link.
 */
typedef struct FrameList {
    FrameInfo *list;
    FrameInfo *end;
} FrameList;

static void frame_list_clear(FrameList *frame_list)
{
    if (frame_list) {
        while (frame_list->list) {
            FrameInfo *info = frame_list->list;
            frame_list->list = info->next;
            if(info->frame) {
            	av_frame_free(&info->frame);
			}
            av_free(info);
        }
        frame_list->end        = NULL;
    }
}


static int64_t frame_list_next_pts(FrameList *frame_list)
{
    if (!frame_list->list || frame_list->list->pts == AV_NOPTS_VALUE)
        return AV_NOPTS_VALUE;
	
    return frame_list->list->pts + frame_list->list->offset;
}

static int64_t frame_list_last_pts(FrameList *frame_list)
{
    if (!frame_list->list || !frame_list->end)
        return AV_NOPTS_VALUE;

	return frame_list->end->pts + frame_list->end->length_pts;
}





typedef struct MixContext {
    const AVClass *class;       /**< class for AVOptions */
    AVFloatDSPContext *fdsp;

    int nb_inputs;              /**< number of inputs */
    int active_inputs;          /**< number of input currently active */
    int duration_mode;          /**< mode for determining duration */
    float dropout_transition;   /**< transition time when an input drops out */

    int nb_channels;            /**< number of channels */
    int sample_rate;            /**< sample rate */
    int planar;
    uint8_t *input_state;       /**< current state of each input */
    float *input_scale;         /**< mixing scale factor for each input */
    float scale_norm;           /**< normalization factor for all inputs */
    FrameList **inputs_frame_list;      /**< list of frame info for the first input */
	int64_t last_output_pts; //last generated pts
	int64_t max_cache_pts;
} MixContext;

#define OFFSET(x) offsetof(MixContext, x)
#define A AV_OPT_FLAG_AUDIO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM
static const AVOption amix_options[] = {
    { "inputs", "Number of inputs.",
            OFFSET(nb_inputs), AV_OPT_TYPE_INT, { .i64 = 2 }, 1, 32, A|F },
    { "duration", "How to determine the end-of-stream.",
            OFFSET(duration_mode), AV_OPT_TYPE_INT, { .i64 = DURATION_LONGEST }, 0,  2, A|F, "duration" },
        { "longest",  "Duration of longest input.",  0, AV_OPT_TYPE_CONST, { .i64 = DURATION_LONGEST  }, INT_MIN, INT_MAX, A|F, "duration" },
        { "shortest", "Duration of shortest input.", 0, AV_OPT_TYPE_CONST, { .i64 = DURATION_SHORTEST }, INT_MIN, INT_MAX, A|F, "duration" },
        { "first",    "Duration of first input.",    0, AV_OPT_TYPE_CONST, { .i64 = DURATION_FIRST    }, INT_MIN, INT_MAX, A|F, "duration" },
    { "dropout_transition", "Transition time, in seconds, for volume "
                            "renormalization when an input stream ends.",
            OFFSET(dropout_transition), AV_OPT_TYPE_FLOAT, { .dbl = 2.0 }, 0, INT_MAX, A|F },
    { NULL }
};

AVFILTER_DEFINE_CLASS(amix);

static int frame_list_add_frame(FrameList *frame_list, AVFrame *frame, int64_t pts, MixContext *s, AVFilterLink *outlink, int stream_idx)
{
	FrameInfo *info, **next_frame, *prev_frame = NULL;
	int64_t frame_length_pts = av_rescale_q(frame->nb_samples, (AVRational){ 1, outlink->sample_rate }, outlink->time_base);
	int64_t frame_end_pts = pts + frame_length_pts;
	

	if (frame_list->list && (frame_end_pts - (frame_list->list->pts + frame_list->list->offset) > s->max_cache_pts)) {
		av_log(NULL, AV_LOG_ERROR,
			"amix frame_list_add_frame stream %d reached cache buffer limit (cache start pts: %ld, last frame end pts: %ld, limit in pts: %ld, limit in seconds: %d)!\n",
			stream_idx,
			frame_list->list->pts + frame_list->list->offset,
			frame_end_pts,
			s->max_cache_pts,
			MAX_CACHE_SECONDS
			);
		return AVERROR(ENOMEM);
	}

	//find the previous (related to this one) frame considenring that in udp some frames can arrive later than expected
	next_frame = &(frame_list->list);
	while (*next_frame) {
		if ((pts < (*next_frame)->pts))  {
			break;
		}
		prev_frame = *next_frame;
		next_frame = &((*next_frame)->next);
	}


	//does the prev_frame ends after the end of this frame  (total overlap)?
	//queue: |----| |----|
	//prev     ^
	//next             ^
	//frame    |--|      
	if (prev_frame && ((prev_frame)->pts + (prev_frame)->length_pts >= pts + frame_length_pts)) {
		//discard the whole frame!
		av_log(NULL, AV_LOG_WARNING,
			"amix frame_list_add_frame stream %d discarding whole frame beacuse end pts %ld is less than prev frame end pts %ld!\n",
			stream_idx,
			pts + frame_length_pts,
			(prev_frame)->pts + (prev_frame)->length_pts
			);
		return 0;
	}


	//does the next_frame ends before the end of this frame  (total overlap)?
	//queue: |----|      |----|
	//prev     ^
	//next                  ^
	//frame           |---------|   
	if ((*next_frame) && ((*next_frame)->pts + (*next_frame)->length_pts <= pts + frame_length_pts)) {
		//discard the whole frame!
		av_log(NULL, AV_LOG_WARNING,
			"amix frame_list_add_frame stream %d discarding whole frame beacuse end pts %ld is greater than next frame end pts %ld!\n",
			stream_idx,
			pts + frame_length_pts,
			(*next_frame)->pts + (*next_frame)->length_pts
			);
		return 0;
	}

	info = av_malloc(sizeof(*info));
	if (!info) {
		av_log(NULL, AV_LOG_ERROR, "amix frame_list_add_frame CANNOT ALLOCATE FRAME_INFO !!!!\n");
		return AVERROR(ENOMEM);
	}
	info->nb_samples = frame->nb_samples;
	info->length_pts = frame_length_pts;
	info->pts = pts;
	info->next = NULL;
	
	if (*next_frame != NULL) {
		av_log(NULL, AV_LOG_INFO, "amix frame_list_add_frame stream %d adding frame in the middle (this pts: %ld, this pts_end: %ld, next pts: %ld) ...\n", stream_idx, pts, pts + frame_length_pts, (*next_frame)->pts);
		//there is a frame next to this one eg:
		//queue:   |-----|      |------|
		//info:         |-----|
		//next_frame            ^
		info->next = *next_frame;
		//her we are updating the next field of the preceding frame or the head of the list
		(*next_frame) = info;
	}
	else {
		//there is no frame next to this one eg:
		//queue:    |-----|  |------|
		//info:                         |----|
		//next_frame  NULL
		//or 
		//queue: NULL
		//info:  |----|
		//next_frame  NULL

		*next_frame = info;
		frame_list->end = info;
	}
	info->offset = 0;
	info->frame = av_frame_clone(frame);

	if (!info->frame) {
		av_log(NULL, AV_LOG_ERROR, "amix frame_list_add_frame CANNOT ALLOCATE INFO->FRAME !!!!\n");
		return AVERROR(ENOMEM);
	}


	av_assert0(frame_list->end);

	if (prev_frame != NULL && prev_frame->pts + prev_frame->length_pts > info->pts) {
		//av_log(NULL, AV_LOG_INFO, "START1PREV amix frame_list_add_frame pts: %ld ...\n", pts);
		//if previous frame overlaps the start of this frame
		//prev_frame |----------|
		//info             |------|
		//offset            XXX
		info->offset = prev_frame->pts + prev_frame->length_pts - info->pts;

		if (info->offset >= info->length_pts) {
			//prev_frame |--------------|
			//info             |-----|
			//offset           XXXXXXXXXX
			av_log(NULL, AV_LOG_ERROR,
				"amix frame_list_add_frame stream %d UNEXPECTED FRAME TOTAL OVERLAPPING (%ld samples of %ld) beacuse start pts %ld is less than prev_frame end pts %ld (prev_frame start is %ld, length is %ld)!\n",
				stream_idx,
				info->offset,
				frame_length_pts,
				pts,
				(prev_frame->pts + prev_frame->length_pts),
				prev_frame->pts,
				prev_frame->length_pts
				);

			//normalize offset to be not greater than frame length
			info->offset = info->length_pts;
		} else if (info->offset > 2) {
			av_log(NULL, AV_LOG_WARNING,
				"amix frame_list_add_frame stream %d discarding partial of this frame (%ld samples of %ld) beacuse start pts %ld is less than prev_frame end pts %ld (prev_frame start is %ld, length is %ld)!\n",
				stream_idx,
				info->offset,
				frame_length_pts,
				pts,
				(prev_frame->pts + prev_frame->length_pts),
				prev_frame->pts,
				prev_frame->length_pts
				);
		}

	}

	if (info->next && info->next->pts < info->pts + info->length_pts) {
		//if next frame overlaps the end of this frame
		//next_frame       |-----|
		//info         |------|
		//nextoffset        XXX
		info->next->offset = info->pts + info->length_pts - info->next->pts;
		
		if (info->next->offset >= info->next->length_pts) {
			//next_frame       |-----|
			//info           |---------|
			//nextoffset        XXXXXXXX
			av_log(NULL, AV_LOG_ERROR,
				"amix frame_list_add_frame stream %d UNEXPECTED FRAME TOTAL OVERLAPPING (%ld samples of %ld) beacuse next_frame start pts %ld is less than this frame end pts %ld (this frame start is %ld, length is %ld)!\n",
				stream_idx,
				info->next->offset,
				info->next->length_pts,
				info->next->pts,
				(info->pts + info->length_pts),
				info->pts,
				info->length_pts
				);
			//normalize offset to be not greater than frame length
			info->next->offset = info->next->length_pts;
		} else if (info->next->offset > 2) {
			av_log(NULL, AV_LOG_WARNING,
				"amix frame_list_add_frame stream %d discarding partial of next frame (%ld samples of %ld) beacuse next_frame start pts %ld is less than this frame end pts %ld (this frame start is %ld, length is %ld)!\n",
				stream_idx,
				info->next->offset,
				info->next->length_pts,
				info->next->pts,
				(info->pts + info->length_pts),
				info->pts,
				info->length_pts
				);
		}
	}
	else if (info->next) {
		info->next->offset = 0;
	}

	return 0;
}



/**
 * Update the scaling factors to apply to each input during mixing.
 *
 * This balances the full volume range between active inputs and handles
 * volume transitions when EOF is encountered on an input but mixing continues
 * with the remaining inputs.
 */
static void calculate_scales(MixContext *s)
{
    int i;

    s->scale_norm = s->active_inputs;

    for (i = 0; i < s->nb_inputs; i++) {
        if (s->input_state[i] == INPUT_ON)
            s->input_scale[i] = 1.0f / s->scale_norm;
        else
            s->input_scale[i] = 0.0f;
    }
}

static int config_output(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MixContext *s      = ctx->priv;
    int i;
    char buf[64];

	s->last_output_pts = AV_NOPTS_VALUE;
    s->planar          = av_sample_fmt_is_planar(outlink->format);
    s->sample_rate     = outlink->sample_rate;
    outlink->time_base = (AVRational){ 1, outlink->sample_rate };
	outlink->flags |= FF_LINK_FLAG_REQUEST_LOOP;
	s->max_cache_pts = MAX_CACHE_SECONDS * outlink->sample_rate;

    s->inputs_frame_list = av_mallocz_array(s->nb_inputs, sizeof(*s->inputs_frame_list));
    if (!s->inputs_frame_list)
        return AVERROR(ENOMEM);


    s->nb_channels = av_get_channel_layout_nb_channels(outlink->channel_layout);
    for (i = 0; i < s->nb_inputs; i++) {
        s->inputs_frame_list[i] =av_mallocz(sizeof(*s->inputs_frame_list[i]));
		if (!s->inputs_frame_list[i])
            return AVERROR(ENOMEM);
    }

    s->input_state = av_malloc(s->nb_inputs);
    if (!s->input_state)
        return AVERROR(ENOMEM);
    memset(s->input_state, INPUT_ON, s->nb_inputs);
    s->active_inputs = s->nb_inputs;

    s->input_scale = av_mallocz_array(s->nb_inputs, sizeof(*s->input_scale));
    if (!s->input_scale)
        return AVERROR(ENOMEM);
    s->scale_norm = s->active_inputs;
    calculate_scales(s);

    av_get_channel_layout_string(buf, sizeof(buf), -1, outlink->channel_layout);

    av_log(ctx, AV_LOG_VERBOSE,
           "inputs:%d fmt:%s srate:%d cl:%s\n", s->nb_inputs,
           av_get_sample_fmt_name(outlink->format), outlink->sample_rate, buf);

    return 0;
}



/**
 * Calculates the number of active inputs and determines EOF based on the
 * duration option.
 *
 * @return 0 if mixing should continue, or AVERROR_EOF if mixing should stop.
 */
static int calc_active_inputs(MixContext *s)
{
    int i;
    int active_inputs = 0;
    for (i = 0; i < s->nb_inputs; i++)
        active_inputs += !!(s->input_state[i] != INPUT_OFF);
    s->active_inputs = active_inputs;

    if (!active_inputs ||
        (s->duration_mode == DURATION_FIRST && s->input_state[0] == INPUT_OFF) ||
        (s->duration_mode == DURATION_SHORTEST && active_inputs != s->nb_inputs))
        return AVERROR_EOF;
    return 0;
}

inline static void _vector_fmac_scalar_c(int16_t *dst, const int16_t *src, float mul,
	int len)
{
	int i;
	for (i = 0; i < len; i++)
		dst[i] += src[i] * mul;
}

static int request_frame(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    MixContext      *s = ctx->priv;
    int ret, i;
	int wanted_samples, corrected_pts_start = 0;
    int64_t pts_start = INT64_MAX, pts_end = INT64_MAX, cur_pts, this_stream_pts_start, this_stream_pts_end;
	AVFrame *mixed_frame;
	AVFrame *input_merged_frame;


    calc_active_inputs(s);
    if (s->active_inputs != s->nb_inputs) {
		av_log(ctx, AV_LOG_ERROR, "amix request_frame but at least one of the active inputs is OFF\n");
		return AVERROR_EOF;
	}

	//lets check if all the inputs have at least one frame
	for (i = 0; i < ctx->nb_inputs; i++) {
		cur_pts = this_stream_pts_start = frame_list_next_pts(s->inputs_frame_list[i]);
		if (cur_pts == AV_NOPTS_VALUE) {
			av_log(ctx, AV_LOG_DEBUG, "amix request_frame but input idx %d has no frames. trying to request frames ...\n", i);
			ret = ff_request_frame(ctx->inputs[i]);
			if (ret == AVERROR_EOF) {
				av_log(ctx, AV_LOG_ERROR, "amix request_frame but input idx %d has no frames and returned EOF on request frames!\n", i);
				s->input_state[i] = INPUT_OFF;
				if (s->nb_inputs == 1)
					return AVERROR_EOF;
				else
					return AVERROR(EAGAIN);
			} else if (ret == AVERROR(EAGAIN)) {
				//no data still available ...
				return ret;
			}
			else if (ret < 0) {
				av_log(ctx, AV_LOG_ERROR, "amix request_frame but input idx %d has no frames and returned %d on request frames!\n", i, ret);
            	return ret;
			}
			cur_pts = this_stream_pts_start = frame_list_next_pts(s->inputs_frame_list[i]);
		}
		
		if (cur_pts != AV_NOPTS_VALUE && cur_pts < pts_start) {
			pts_start = cur_pts;
		}
		
		cur_pts = this_stream_pts_end = frame_list_last_pts(s->inputs_frame_list[i]);
		/*while (1000LL * (this_stream_pts_end - this_stream_pts_start) / s->sample_rate < MIN_CACHE_MILLISECONDS) {
			//av_log(ctx, AV_LOG_INFO, "amix request_frame but input idx %d has not enough frames. trying to request frames ...\n", i);
			ret = ff_request_frame(ctx->inputs[i]);
			if (ret == AVERROR_EOF) {
				av_log(ctx, AV_LOG_ERROR, "amix request_frame but input idx %d has no frames and returned EOF on request frames!\n", i);
				s->input_state[i] = INPUT_OFF;
				if (s->nb_inputs == 1)
					return AVERROR_EOF;
				else
					return AVERROR(EAGAIN);
			}
			else if (ret == AVERROR(EAGAIN)) {
				//no data still available ...
				return ret;
			}
			else if (ret < 0) {
				av_log(ctx, AV_LOG_ERROR, "amix request_frame but input idx %d has no frames and returned %d on request frames!\n", i, ret);
				return ret;
			}
			cur_pts = this_stream_pts_end = frame_list_last_pts(s->inputs_frame_list[i]);
		}*/
		if (cur_pts != AV_NOPTS_VALUE && cur_pts < pts_end) {
					pts_end = cur_pts;
		}
	}
	//av_log(ctx, AV_LOG_WARNING, "amix request_frame seems that all inputs have some frames ...\n");

	if (pts_end == INT64_MAX || pts_start == INT64_MAX || pts_start >= pts_end) {
		av_log(ctx, AV_LOG_ERROR, "amix request_frame but global pts_start %ld is not coherent with pts_end %ld\n", pts_start, pts_end);
		return AVERROR(EAGAIN);
	}

	//to allow the arriving of old packets we keep a cache of at least CONSUME_CACHE_MILLISECONDS (MIN_CACHE_MILLISECONDS = 2*CONSUME_CACHE_MILLISECONDS)
	/*if (1000LL * (pts_end - pts_start) / s->sample_rate < MIN_CACHE_MILLISECONDS) {
		av_log(ctx, AV_LOG_ERROR, "amix request_frame but queues contains only %lld ms instead of %lld ms\n", 1000LL * (pts_end - pts_start) / s->sample_rate, MIN_CACHE_MILLISECONDS);
		return AVERROR(EAGAIN);
	}
	av_log(ctx, AV_LOG_INFO, "amix request_frame updating pts_end %ld to %ld ...\n", pts_end, (int64_t)( pts_end - (int64_t)(CONSUME_CACHE_MILLISECONDS * s->sample_rate) / (int64_t)1000LL));
	pts_end = pts_end - (int64_t)(CONSUME_CACHE_MILLISECONDS * s->sample_rate) / (int64_t)1000LL;

	if (pts_start >= pts_end) {
		av_log(ctx, AV_LOG_ERROR, "amix request_frame but update of pts_end to keep at least CONSUME_CACHE_MILLISECONDS set a not coherent pts_end %ld with pts_start %ld\n", pts_end, pts_start);
		return AVERROR(EAGAIN);
	}*/

	//in case we have lost frames on all the input channels we must insert silence also for the corresponding
	// window.
	//eg.
	// stream1   |XXXXXXX|    |---------|
	// stream2 |XXXXXXXXX|   |------|
	// pts_start             ^
	// pts_end                      ^
	// last_output_pts   ^
	// new window        ^----------^
	if (pts_start > s->last_output_pts && s->last_output_pts != AV_NOPTS_VALUE) {
		av_log(ctx, AV_LOG_ERROR, "amix request_frame but global last_output_pts %ld is less than one of the queue pts_start %ld. This can happen if we have lost some packets... Generating silence for %ld samples ...\n", s->last_output_pts, pts_start, (pts_start - s->last_output_pts));
		pts_start = s->last_output_pts;
		corrected_pts_start = 1;
	} else if (pts_start < s->last_output_pts && s->last_output_pts != AV_NOPTS_VALUE) {
		if (s->last_output_pts - pts_start > 2) {
			av_log(ctx, AV_LOG_INFO, "amix request_frame but global last_output_pts %ld is later than one of the queue pts_start %ld. This can happen if we have received some packets too late or received an RTCP NTP update on an instable stream. Discarding the too late part ...\n", s->last_output_pts, pts_start);
		}
		pts_start = s->last_output_pts;
		corrected_pts_start = 1;
	}
    wanted_samples = pts_end - pts_start;
	if (wanted_samples <= 0) {
		if (corrected_pts_start) {
			av_log(ctx, AV_LOG_WARNING, "amix request_frame but global pts_end %ld is less than pts_start %ld due to the correction of pts_start with last_output_pts %ld. This can happen if we have received some packets too late or received an RTCP NTP update on an instable stream. Clearing samples up to last_output_pts ...\n", pts_end, pts_start, s->last_output_pts);
			pts_end = pts_start;
			wanted_samples = pts_end - pts_start;
		}
		else {
			av_log(ctx, AV_LOG_ERROR, "amix request_frame but global pts_end %ld is less than pts_start %ld but no correction was performed (last_output_pts %ld). This can happen if the queue sequence is broken!!! Exiting with ENOMEM...\n", pts_end, pts_start, s->last_output_pts);
			return AVERROR(ENOMEM);
		}
	}


 
	//calculate scaling factor needed to mix audios
	calculate_scales(s);

	if (wanted_samples > 0) {
		mixed_frame = ff_get_audio_buffer(outlink, wanted_samples);
		if (!mixed_frame) {
			av_log(ctx, AV_LOG_ERROR, "amix request_frame CANNOT ALLOCATE MIXED FRAME (wanted_samples: %d, pts_start: %ld, pts_end: %ld) !!!!\n", wanted_samples, pts_start, pts_end);
			return AVERROR(ENOMEM);
		}
		input_merged_frame = ff_get_audio_buffer(outlink, wanted_samples);
		if (!input_merged_frame) {
			av_log(ctx, AV_LOG_ERROR, "amix request_frame CANNOT ALLOCATE INPUT MERGED FRAME (wanted_samples: %d, pts_start: %ld, pts_end: %ld) !!!!\n", wanted_samples, pts_start, pts_end);
			return AVERROR(ENOMEM);
		}
	}
	

	//ok, now we have to mix all the inputs considering that frames can be not contiguous in a single input and also
	// that each input can start after pts_start, eg:
	//INPUT0 [ 0_9]->[10-19]->[20-49]
	//INPUT1 [20-29]->[30-39]
	//INPUT2 [35-39]
	// pts_start = 0
	// pts_end = 39

	for (i = 0; i < s->nb_inputs; i++) {
		int planes, plane_size, p;
		int64_t samples_copied = 0, missed_msecs = 0;
		//av_log(ctx, AV_LOG_WARNING, "amix request_frame loop input %d to mix ...\n", i);

		cur_pts = frame_list_next_pts(s->inputs_frame_list[i]);
		if (wanted_samples > 0) {
			av_samples_set_silence(input_merged_frame->extended_data, 0, wanted_samples, s->nb_channels, outlink->format);
		}

		//we should use frames of this input until the start of the frame is included in the pts_start<->pts_end window
		while(cur_pts < pts_end && cur_pts != AV_NOPTS_VALUE) {
			//av_log(ctx, AV_LOG_WARNING, "amix request_frame loop input %d to mix cur_pts %ld pts_start %ld pts_end %ld...\n", i, cur_pts, pts_start, pts_end);
			//copy the current frame buffer in the destination buffer considering the offsets
			//eg
			// pts_start 40
			// pts_end   100
			// this_frame->pts_start 20
			// this_frame->pts_end   110
			// out buffer   |-------------------|
			// frame buffer      |XXXXXXXXXXXXXX----|

			FrameInfo *frame_info = s->inputs_frame_list[i]->list;
			AVFrame *frame = frame_info->frame;
			int64_t copy_pts_end = frame_info->pts + frame_info->length_pts;
			int64_t copy_pts_start = cur_pts;
			
			//check if this frame ends after the start of the pts_start<->pts_end window to discard frames of the following case:
			// out buffer            |--------------|
			// frame buffer |-----|
			if (copy_pts_end > pts_start) {
				if (copy_pts_start < pts_start) {
					//check if this frame start before the start of the pts_start<->pts_end window:
					// out buffer      |--------------|
					// frame buffer |---XXXXX|
					copy_pts_start = pts_start;
				}
				if (copy_pts_end > pts_end) {
					//check if this frame ends after the pts_start<->pts_end window:
					// out buffer      |--------------|
					// frame buffer			|XXXXXXXXX-----|
					copy_pts_end = pts_end;
				}
				//copy the samples from the frame to the 
				if (wanted_samples > 0) {
					if (av_samples_copy(input_merged_frame->extended_data, frame->extended_data,
						copy_pts_start - pts_start, //dest offset
						copy_pts_start - frame_info->pts, //src offset
						copy_pts_end - copy_pts_start, //samples
						s->nb_channels, frame->format) < 0) {
						av_log(ctx, AV_LOG_ERROR, "amix request_frame av_samples_copy RETURNED ERROR (copy_pts_start: %ld, copy_pts_end: %ld, pts_start: %ld, frame_info->pts: %ld) !!!!\n", copy_pts_start, copy_pts_end, pts_start, frame_info->pts);
						return AVERROR(ENOMEM);
					}
				}
				samples_copied += copy_pts_end - copy_pts_start;

				if (copy_pts_end >= frame_info->pts + frame_info->length_pts) {
					//we have consumed all the samples in this frame. drop it ...
					// out buffer   |--------------|
					// frame buffer	    |XXXXXXXXX|
					s->inputs_frame_list[i]->list = frame_info->next;
					av_frame_free(&frame_info->frame);
					av_freep(&frame_info);
					if (s->inputs_frame_list[i]->list == NULL) {
						s->inputs_frame_list[i]->end = NULL;
					}
				} else {
					//(copy_pts_end < pts_end) there is still something in the frame. keep it and update the offset ...
					// out buffer   |--------------|
					// frame buffer	         |XXXXX----|
					frame_info->offset = copy_pts_end - frame_info->pts;
				}

			} else {
				//this sample has been totally discarded. drop it ...
				// out buffer            |--------------|
				// frame buffer |----|
				s->inputs_frame_list[i]->list = frame_info->next;
				av_frame_free(&frame_info->frame);
				av_freep(&frame_info);
				if (s->inputs_frame_list[i]->list == NULL) {
					s->inputs_frame_list[i]->end = NULL;
				}
			}

			//update the cur_pts with the new frame head
			cur_pts = frame_list_next_pts(s->inputs_frame_list[i]);
		}
		//here we have merged all the interested frames of this input into input_audio_data
		//now we mix all the planes of input_audio_data into mix_audio_data
		if (wanted_samples > 0) {
			planes = s->planar ? s->nb_channels : 1;
			plane_size = wanted_samples * (s->planar ? 1 : s->nb_channels);
			plane_size = FFALIGN(plane_size, 16);

			for (p = 0; p < planes; p++) {
				/*s->fdsp->vector_fmac_scalar((float *)mixed_frame->extended_data[p], //dest
											(float *)input_merged_frame->extended_data[p], //source
											s->input_scale[i], plane_size);*/
				_vector_fmac_scalar_c((int16_t *)mixed_frame->extended_data[p], //dest
					(int16_t *)input_merged_frame->extended_data[p], //source
					s->input_scale[i], plane_size);
			}

			missed_msecs = (1000 * (wanted_samples - samples_copied)) / s->sample_rate;
			if (missed_msecs > 1) {
				av_log(ctx, AV_LOG_INFO, "amix stream idx %d lost %ld milliseconds of audio...\n", i, missed_msecs);
			}
		}
	}
	//av_log(ctx, AV_LOG_WARNING, "amix request_frame mixed_frame with pts_start %ld !!!!!!\n", pts_start);
	if (wanted_samples > 0) {
		mixed_frame->pts = pts_start;
		s->last_output_pts = pts_end;
		av_frame_free(&input_merged_frame);

		return ff_filter_frame(outlink, mixed_frame);
	}
	else {
		return AVERROR(EAGAIN);
	}

}



static int filter_frame(AVFilterLink *inlink, AVFrame *buf)
{
    AVFilterContext  *ctx = inlink->dst;
    MixContext       *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
	int64_t pts;
    int i, ret = 0;

    for (i = 0; i < ctx->nb_inputs; i++)
        if (ctx->inputs[i] == inlink)
            break;
    if (i >= ctx->nb_inputs) {
        av_log(ctx, AV_LOG_ERROR, "unknown input link\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }

	//av_log(ctx, AV_LOG_INFO, "amix filter_frame adding frame input idx: %d, orig pts: %ld, pts: %ld, samples: %d, length_in_pts %ld\n", i, buf->pts, av_rescale_q(buf->pts, inlink->time_base, outlink->time_base), buf->nb_samples, av_rescale_q(buf->nb_samples, (AVRational){ 1, inlink->sample_rate }, inlink->time_base));

	pts = av_rescale_q(buf->pts, inlink->time_base,
							   outlink->time_base);
	if (pts == 0) {
		av_log(ctx, AV_LOG_WARNING, "amix filter_frame input idx %d adding frame with rescaled pts %ld (buf->pts = %ld)!\n", i, pts, buf->pts);
	}
	ret = frame_list_add_frame(s->inputs_frame_list[i], buf, pts, s, outlink, i);
	if (ret < 0)
		goto fail;

fail:
    av_frame_free(&buf);

    return ret;
}

static av_cold int init(AVFilterContext *ctx)
{
    MixContext *s = ctx->priv;
    int i;

    for (i = 0; i < s->nb_inputs; i++) {
        char name[32];
        AVFilterPad pad = { 0 };

        snprintf(name, sizeof(name), "input%d", i);
        pad.type           = AVMEDIA_TYPE_AUDIO;
        pad.name           = av_strdup(name);
        if (!pad.name)
            return AVERROR(ENOMEM);
        pad.filter_frame   = filter_frame;

        ff_insert_inpad(ctx, i, &pad);
    }

    s->fdsp = avpriv_float_dsp_alloc(0);
    if (!s->fdsp)
        return AVERROR(ENOMEM);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    int i;
    MixContext *s = ctx->priv;

   
    if (s->inputs_frame_list) {
	        for (i = 0; i < s->nb_inputs; i++) {
	    		frame_list_clear(s->inputs_frame_list[i]);
			}
	        av_freep(&s->inputs_frame_list);
    }
    av_freep(&s->input_state);
    av_freep(&s->input_scale);
    av_freep(&s->fdsp);

    for (i = 0; i < ctx->nb_inputs; i++)
        av_freep(&ctx->input_pads[i].name);
}

static int query_formats(AVFilterContext *ctx)
{
    AVFilterFormats *formats = NULL;
    AVFilterChannelLayouts *layouts;
    int ret;

    layouts = ff_all_channel_layouts();

    if (!layouts)
        return AVERROR(ENOMEM);

	ff_add_format(&formats, AV_SAMPLE_FMT_S16);
	ff_add_format(&formats, AV_SAMPLE_FMT_S16P);
    ret = ff_set_common_formats(ctx, formats);
    if (ret < 0)
        return ret;
    ret = ff_set_common_channel_layouts(ctx, layouts);
    if (ret < 0)
        return ret;
    return ff_set_common_samplerates(ctx, ff_all_samplerates());
}

static const AVFilterPad avfilter_af_amix_outputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_AUDIO,
        .config_props  = config_output,
        .request_frame = request_frame
    },
    { NULL }
};

AVFilter ff_af_amix = {
    .name           = "amix",
    .description    = NULL_IF_CONFIG_SMALL("Audio mixing."),
    .priv_size      = sizeof(MixContext),
    .priv_class     = &amix_class,
    .init           = init,
    .uninit         = uninit,
    .query_formats  = query_formats,
    .inputs         = NULL,
    .outputs        = avfilter_af_amix_outputs,
    .flags          = AVFILTER_FLAG_DYNAMIC_INPUTS,
};
