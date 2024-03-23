#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/samplefmt.h>
#include <libavutil/timestamp.h>
#include <libswscale/swscale.h>

    cv::Mat frame2CvFrame(AVFrame* frame, const std::string& windowName) {
        int width = frame->width;
        int height = frame->height;
        cv::Mat cvFrame = cv::Mat(height, width, CV_8UC3);
        int cvLinesizes[1];
        cvLinesizes[0] = cvFrame.step1();
        SwsContext* conversion = sws_getContext(frame->width, frame->height, (AVPixelFormat)frame->format, frame->width, frame->height, AVPixelFormat::AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR, NULL, NULL, NULL);
        sws_scale(conversion, frame->data, frame->linesize, 0, frame->height, &cvFrame.data, cvLinesizes);
        sws_freeContext(conversion);

        //cv::imshow(windowName, cvFrame);
        //cv::waitKey(100);

        return cvFrame;
    }

    AVFrame* cvFrame2frame(cv::Mat& cvFrame) {
        int width = cvFrame.cols;
        int height = cvFrame.rows;

        int cvLinesizes[1];
        cvLinesizes[0] = cvFrame.step1();

        AVFrame* frame = av_frame_alloc();
        if (!frame) {
            std::cout << "alloc frame failed " << std::endl;
            return frame;
        }
        frame->width = width;
        frame->height = height;
        frame->format = /*AV_PIX_FMT_BGR24;*/ AV_PIX_FMT_YUV420P;
        frame->pts = 0;
        frame->best_effort_timestamp = 0;
        frame->time_base.num = 1;
        frame->time_base.den = 30000;
        frame->sample_aspect_ratio.num = 1;
        frame->sample_aspect_ratio.den = 1;

        int ret = av_frame_get_buffer(frame, 32);
        if (ret < 0) {
            std::cout << "get buffer failed" << std::endl;
        }
        ret = av_frame_make_writable(frame);
        if (ret < 0) {
            std::cout << " writable failed " << std::endl;
        }

        //cv::cvtColor(cvFrame, cvFrame, cv::COLOR_BGR2YUV_I420);
        //int frame_size = width * height;
        //unsigned char* data = cvFrame.data;
        //memcpy(frame->data[0], data, frame_size);
        //memcpy(frame->data[1], data + frame_size, frame_size / 4);
        //memcpy(frame->data[2], data + frame_size * 5 / 4, frame_size / 4);

        SwsContext* conversion = sws_getContext(width, height, AVPixelFormat::AV_PIX_FMT_BGR24, width, height, (AVPixelFormat)frame->format, SWS_FAST_BILINEAR, NULL, NULL, NULL);
        sws_scale(conversion, &cvFrame.data, cvLinesizes, 0, height, frame->data, frame->linesize);
        sws_freeContext(conversion);
        return frame;
    }

    static AVFormatContext* fmt_ctx;
    static AVCodecContext* dec_ctx;
    AVFilterContext* buffersink_ctx;
    AVFilterContext* buffersrc_ctx;
    AVFilterGraph* filter_graph;
    static int video_stream_index = -1;
    static int64_t last_pts = AV_NOPTS_VALUE;

    int imageFilter(cv::Mat &cvFrame) {
        AVFrame* frame = cvFrame2frame(cvFrame);
        AVFrame* filt_frame = av_frame_alloc();
        if (!frame || !filt_frame) {
            perror("Could not allocate frame");
            exit(1);
        }

        char args[512];
        memset(args, 0, sizeof(args));
        int ret;
        const char* filter_descr = "fftfilt=dc_Y=0:weight_Y=\'1+squish(1-(Y+X)/100)\'[out]"; // "fftdnoiz[out]";

        const AVFilter* buffersrc = avfilter_get_by_name("buffer");
        const AVFilter* buffersink = avfilter_get_by_name("buffersink");

        AVFilterInOut* inputs = avfilter_inout_alloc();
        AVFilterInOut* outputs = avfilter_inout_alloc();
        if (!inputs || !outputs)
        {
            printf("Cannot alloc input / output\n");
            return -1;
        }

        AVFilterGraph* filter_graph = avfilter_graph_alloc();
        if (!filter_graph)
        {
            printf("Cannot create filter graph\n");
            return -1;
        }

        /* buffer video source: the decoded frames from the decoder will be inserted here. */
        snprintf(args, sizeof(args),
            "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
            frame->width, frame->height, frame->format,
            frame->time_base.num, frame->time_base.den,
            frame->sample_aspect_ratio.num, frame->sample_aspect_ratio.den);

        ret = avfilter_graph_create_filter(&buffersrc_ctx, buffersrc, "in",
            args, NULL, filter_graph);
        if (ret < 0) {
            printf("Cannot create buffer source\n");
            return ret;
        }

        /* buffer video sink: to terminate the filter chain. */
        ret = avfilter_graph_create_filter(&buffersink_ctx, buffersink, "out",
            NULL, NULL, filter_graph);
        if (ret < 0) {
            printf("Cannot create buffer sink\n");
            return ret;
        }

        /* Endpoints for the filter graph. */
        outputs->name = av_strdup("in");
        outputs->filter_ctx = buffersrc_ctx;
        outputs->pad_idx = 0;
        outputs->next = NULL;

        inputs->name = av_strdup("out");
        inputs->filter_ctx = buffersink_ctx;
        inputs->pad_idx = 0;
        inputs->next = NULL;

        if ((ret = avfilter_graph_parse_ptr(filter_graph, filter_descr,
            &inputs, &outputs, NULL)) < 0)
            return ret;

        if ((ret = avfilter_graph_config(filter_graph, NULL)) < 0)
            return ret;

        avfilter_inout_free(&inputs);
        avfilter_inout_free(&outputs);

        if (av_buffersrc_add_frame(buffersrc_ctx, frame) < 0) {
            printf("Error while feeding the filtergraph\n");
            return -1;
        }

        while (1) {
            ret = av_buffersink_get_frame_flags(buffersink_ctx, filt_frame, 0);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            if (ret < 0)
                return ret;

            if (filt_frame) {
                cvFrame = frame2CvFrame(filt_frame, "filter");
            }
            av_frame_unref(filt_frame);
        }
        av_frame_unref(frame);
        avfilter_graph_free(&filter_graph);
        
        return 0;
    }

    int testFilter() {
        std::string sInName = "E:/projects/instant-ngp/data/nerf/lab_back/images/1.jpg";
        cv::Mat cvFrame = cv::imread(sInName);
        return imageFilter(cvFrame);
    }
}