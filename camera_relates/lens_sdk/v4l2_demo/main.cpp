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

int main()
{
    //打开摄像头设备
    int fd_video = open("/dev/video0", O_RDWR);
    if (fd_video < 0) {
        perror("video capture open");
        return fd_video;
    }

    //打印支持的视频格式
    struct v4l2_fmtdesc fmtdesc;
    fmtdesc.index=0;
    fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    printf("Support format:\n");
    while(ioctl(fd_video, VIDIOC_ENUM_FMT, &fmtdesc) != -1) {
        printf("\t%d.%s ,fmtdesc:%d\n", fmtdesc.index + 1, fmtdesc.description, fmtdesc.pixelformat);
        fmtdesc.index++;
    }

    struct v4l2_format s_fmt;
    s_fmt.fmt.pix.width  = 640;
    s_fmt.fmt.pix.height = 480;
//    s_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; // 设置视频的格式，720P JPEG
    s_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    printf("s_fmt.fmt.pix.pixelformat:%d\n",s_fmt.fmt.pix.pixelformat);
    s_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int flag= ioctl(fd_video, VIDIOC_S_FMT, &s_fmt);
    if(flag != 0) {
        printf("set format error\n");
        return -1;
    }
    printf("image size:%d %d\n", s_fmt.fmt.pix.width, s_fmt.fmt.pix.height);

    //申请1个缓冲区
    struct v4l2_requestbuffers req;
    req.count=1;
    req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory=V4L2_MEMORY_MMAP;
    ioctl(fd_video, VIDIOC_REQBUFS, &req);

    //缓冲区与应用程序关联
    //申请1个struct buffer空间
    struct buffer *buffers = (struct buffer*)calloc (req.count, sizeof (struct buffer));
    if (!buffers) {
        perror("Out of memory");
        exit(EXIT_FAILURE);
    }

    for (int n_buffers = 0; n_buffers < req.count; n_buffers++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = n_buffers;
        if (-1 == ioctl(fd_video, VIDIOC_QUERYBUF, &buf))
            exit(-1);
        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start  = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_video, buf.m.offset);
        if (MAP_FAILED == buffers[n_buffers].start)
            exit(-1);
    }

    for (int n_buffers = 0; n_buffers < req.count; n_buffers++) {
        struct v4l2_buffer buf;
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;
        ioctl(fd_video, VIDIOC_QBUF, &buf);
    }

    //开始捕获图像
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl (fd_video, VIDIOC_STREAMON, &type);

    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    //取出图像数据
    ioctl (fd_video, VIDIOC_DQBUF, &buf);

#if 0
    {
        // save image
        FILE *fp = fopen("/home/cg/test.yuv", "wb+");
        if (fp < 0) {
            perror("fb open error.");
            return -1;
        }
        fwrite(buffers[buf.index].start, 1, buffers[buf.index].length, fp);
        fflush(fp);
        fclose(fp);
    }
#endif

#if 1
    {
        // YUV 2 RGB
        const int width  = 640;
        const int height = 480;
        cv::Mat mat_src = cv::Mat(height, width, CV_8UC2, buffers[buf.index].start);
        cv::Mat mat_dst = cv::Mat(height, width, CV_8UC3);
        cv::cvtColor(mat_src, mat_dst, cv::COLOR_YUV2BGR_YUYV);
        cv::imshow("mat_dst", mat_dst);
        cv::waitKey(0);
    }
#endif

    //放回缓冲区
    ioctl (fd_video,VIDIOC_QBUF,&buf);
    for (int n_buffers = 0; n_buffers < req.count; n_buffers++)
        munmap(buffers[n_buffers].start, buffers[n_buffers].length);

    free(buffers);
    close(fd_video);

    printf("capture jpg finish..\n");
    return 0;
}