#include "cam_v4l2.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>

namespace cg {

    int CamV4l2::init() {

        //打开摄像头设备
        fd_video_ = open(device_name_.c_str(), O_RDWR);
        if (fd_video_ < 0) {
            perror("ERROR: video capture open");
            return fd_video_;
        }

#if __USER_DEBUG__
        struct v4l2_capability cap;
        int ret = ioctl(fd_video_, VIDIOC_QUERYCAP, &cap); // 查看设备功能
        if (ret < 0) {
            perror("requre VIDIOC_QUERYCAP fialed! \n");
            return -1;
        }
        printf("driver:\t\t%s\n", cap.driver);
        printf("card:\t\t%s\n", cap.card);
        printf("bus_info:\t%s\n", cap.bus_info);
        printf("version:\t%d\n", cap.version);
        printf("capabilities:\t%x\n", cap.capabilities);
        if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE) {
            printf("Device supports capture.\n");
        }
        if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING) {
            printf("Device supports streaming.\n");
        }
#endif

#if __USER_DEBUG__
        //打印支持的视频格式
        struct v4l2_fmtdesc fmtdesc;
        fmtdesc.index = 0;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("Support format:\n");
        while (ioctl(fd_video_, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
            printf("\t%d.%s ,fmtdesc: %d\n", fmtdesc.index + 1, fmtdesc.description, fmtdesc.pixelformat);
            fmtdesc.index++;
        }
#endif

        // 设置摄像头拍摄的格式
        struct v4l2_format s_fmt;
        s_fmt.fmt.pix.width  = img_w_;
        s_fmt.fmt.pix.height = img_h_;
        s_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // V4L2_PIX_FMT_MJPEG, 720P JPEG
        s_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        int flag = ioctl(fd_video_, VIDIOC_S_FMT, &s_fmt);
        if (flag != 0) {
            printf("set format error\n");
            return -1;
        }
#if __USER_DEBUG__
        if (-1 == ioctl(fd_video_, VIDIOC_G_FMT, &s_fmt)) {//得到图片格式
            perror("get format failed!");
            return -1;
        }
        printf("fmt.type:\t\t%d\n", s_fmt.type);
        printf("pix.pixelformat:\t%c%c%c%c\n", \
      s_fmt.fmt.pix.pixelformat & 0xFF, \
      (s_fmt.fmt.pix.pixelformat >> 8) & 0xFF, \
      (s_fmt.fmt.pix.pixelformat >> 16) & 0xFF, \
      (s_fmt.fmt.pix.pixelformat >> 24) & 0xFF);
        printf("pix.width:\t\t%d\n", s_fmt.fmt.pix.width);
        printf("pix.height:\t\t%d\n", s_fmt.fmt.pix.height);
        printf("pix.field:\t\t%d\n", s_fmt.fmt.pix.field);
#endif

        //申请1个缓冲区, 一张图片就需要一帧的缓冲区
        struct v4l2_requestbuffers req;
        req.count = n_req_buf_;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        ioctl(fd_video_, VIDIOC_REQBUFS, &req);

        // 缓冲区与应用程序关联, 申请1个struct buffer空间
        buffers_ = (struct buffer *) calloc(req.count, sizeof(struct buffer));
        if (!buffers_) {
            perror("Out of memory");
            exit(EXIT_FAILURE);
        }

        // 用mmap映射内存的目的，是省去了memcpy的花销，更加合理
        for (int i = 0; i < req.count; i++) {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = (unsigned int) i;
            if (-1 == ioctl(fd_video_, VIDIOC_QUERYBUF, &buf))
                exit(-1);
            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_video_, buf.m.offset);
            if (MAP_FAILED == buffers_[i].start)
                exit(-1);
        }

        for (int i = 0; i < req.count; i++) {
            struct v4l2_buffer buf;
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = (unsigned int) i;
            ioctl(fd_video_, VIDIOC_QBUF, &buf);
        }

        //开始捕获图像
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_video_, VIDIOC_STREAMON, &type);

        memset(&(buf_), 0, sizeof(buf_));
        buf_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_.memory = V4L2_MEMORY_MMAP;

        mat_src_ = cv::Mat(img_h_, img_w_, CV_8UC2); // yuv422 image
        mat_dst_ = cv::Mat(img_h_, img_w_, CV_8UC3);
    }

    cv::Mat CamV4l2::read() {
        ioctl(fd_video_, VIDIOC_DQBUF, &buf_); //取出图像数据

        // save("/home/cg/test.yuv");

        // YUV 2 RGB
        mat_src_.data = (uchar*)buffers_[buf_.index].start;
        cv::cvtColor(mat_src_, mat_dst_, cv::COLOR_YUV2BGR_YUYV);

        ioctl(fd_video_, VIDIOC_QBUF, &buf_); //放回缓冲区

        return mat_dst_;
    }

    int CamV4l2::save(std::string file_name) {
        FILE *fp = fopen(file_name.c_str(), "wb+");
        if (fp < 0) {
            perror("fb open error.");
            return -1;
        }
        fwrite(buffers_[buf_.index].start, 1, buffers_[buf_.index].length, fp);
        fflush(fp);
        fclose(fp);
    }

    int CamV4l2::free() {
        for (int i = 0; i < n_req_buf_; i++)
            munmap(buffers_[i].start, buffers_[i].length);

        ::free(buffers_);
        close(fd_video_);
    }

}