//
// Created by cg on 3/11/19.
//

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct buffer
{
    void* start;
    unsigned int length;
};

#define __USER_DEBUG__ 1

int main()
{
    const char* device_name = "/dev/video0";

    //打开摄像头设备
    int fd_video = open(device_name, O_RDWR);
    if (fd_video < 0) {
        perror("video capture open");
        return fd_video;
    }

#if __USER_DEBUG__
    struct v4l2_capability cap;
    int ret = ioctl(fd_video, VIDIOC_QUERYCAP, &cap); // 查看设备功能
    if (ret < 0){
        perror("requre VIDIOC_QUERYCAP fialed! \n");
        return -1;
    }
    printf("driver:\t\t%s\n",cap.driver);
    printf("card:\t\t%s\n",cap.card);
    printf("bus_info:\t%s\n",cap.bus_info);
    printf("version:\t%d\n",cap.version);
    printf("capabilities:\t%x\n",cap.capabilities);
    if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == V4L2_CAP_VIDEO_CAPTURE){
        printf("Device supports capture.\n");
    }
    if ((cap.capabilities & V4L2_CAP_STREAMING) == V4L2_CAP_STREAMING){
        printf("Device supports streaming.\n");
    }
#endif

#if __USER_DEBUG__
    //打印支持的视频格式
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index=0;
    fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while(ioctl(fd_video, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        printf("\t%d.%s ,fmtdesc: %d\n", fmtdesc.index + 1, fmtdesc.description, fmtdesc.pixelformat);
        fmtdesc.index++;
    }
#endif

    const int img_w = 640;
    const int img_h = 480;

    // 设置摄像头拍摄的格式
    struct v4l2_format s_fmt;
    s_fmt.fmt.pix.width  = img_w;
    s_fmt.fmt.pix.height = img_h;
    s_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; // V4L2_PIX_FMT_MJPEG, 720P JPEG
    s_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int flag= ioctl(fd_video, VIDIOC_S_FMT, &s_fmt);
    if(flag != 0) {
        printf("set format error\n");
        return -1;
    }
#if __USER_DEBUG__
    if(-1 == ioctl(fd_video, VIDIOC_G_FMT, &s_fmt)){//得到图片格式
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
    req.count  = 1;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ioctl(fd_video, VIDIOC_REQBUFS, &req);

    // 缓冲区与应用程序关联, 申请1个struct buffer空间
    struct buffer *buffers = (struct buffer*)calloc(req.count, sizeof (struct buffer));
    if (!buffers) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }

    // 用mmap映射内存的目的，是省去了memcpy的花销，更加合理
    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = (unsigned int)i;
        if (-1 == ioctl(fd_video, VIDIOC_QUERYBUF, &buf))
            exit(-1);
        buffers[i].length = buf.length;
        buffers[i].start  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_video, buf.m.offset);
        if (MAP_FAILED == buffers[i].start)
            exit(-1);
    }

    for (int i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = (unsigned int)i;
        ioctl(fd_video, VIDIOC_QBUF, &buf);
    }

    //开始捕获图像
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl (fd_video, VIDIOC_STREAMON, &type);

    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    cv::Mat mat_src = cv::Mat(img_h, img_w, CV_8UC2); // yuv422 image
    cv::Mat mat_dst = cv::Mat(img_h, img_w, CV_8UC3);
    while(true) {
        ioctl(fd_video, VIDIOC_DQBUF, &buf); //取出图像数据

#if 0
        // save image
        FILE *fp = fopen("/home/cg/test.yuv", "wb+");
        if (fp < 0) {
            perror("fb open error.");
            return -1;
        }
        fwrite(buffers[buf.index].start, 1, buffers[buf.index].length, fp);
        fflush(fp);
        fclose(fp);
#endif

        // YUV 2 RGB
        mat_src.data = (uchar*)buffers[buf.index].start;
        cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_YUYV);

        cv::imshow("live cam", mat_dst);
        int key = cv::waitKey(30);
        if(key == 27)
            break;

        ioctl(fd_video, VIDIOC_QBUF, &buf); //放回缓冲区
    }

    for (int i = 0; i < req.count; i++)
        munmap(buffers[i].start, buffers[i].length);

    free(buffers);
    close(fd_video);

    return 0;
}