//
// Created by chary on 18-10-22.
//
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h> /* getopt_long() */
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <dlfcn.h>
#include <linux/videodev2.h>
#include "mipi_camera.hpp"

#define CLEAR(x)  memset(&(x), 0, sizeof(x))

using namespace cv;
namespace mipi {
    static void errno_exit(const char *s) {
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
    }

    static int xioctl(int fh, int request, void *arg) {
        int r;
        do {
            r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);
        return r;
    }

    double frame_diff(timeval *time_val0, timeval *time_val1) {
        return
                (time_val1->tv_sec + time_val1->tv_usec * mipi::kMicroScale) -
                (time_val0->tv_sec + time_val0->tv_usec * mipi::kMicroScale);
    }

    Camera::~Camera() {
        close_camera();
    }

// 1280 * 800
    bool Camera::read_frame(timeval *time_val, Mat &img) {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            errno_exit("VIDIOC_DQBUF");

        img = Mat(800, 1280, CV_8UC1, buffers[buf.index].start);
        resize(img, img, Size(320, 200));

        time_val->tv_sec = buf.timestamp.tv_sec;
        time_val->tv_usec = buf.timestamp.tv_usec;

        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");

        return true;
    }

    void Camera::init_mmap(void) {
        struct v4l2_requestbuffers req;
        CLEAR(req);

        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == ioctl(fd, VIDIOC_REQBUFS, &req)) {

            if (EINVAL == errno) {
                fprintf(stderr, "%s does not support "
                                "memory mapping\n", dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_REQBUFS");
            }
        }

        if (req.count < 2) {
            fprintf(stderr, "Insufficient buffer memory on %s\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }

        buffers = (struct buffer *) calloc(req.count, sizeof(*buffers));

        if (!buffers) {
            fprintf(stderr, "Out of memory\n");
            exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
            struct v4l2_buffer buf;
            CLEAR(buf);
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = n_buffers;

            if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                errno_exit("VIDIOC_QUERYBUF");

            buffers[n_buffers].length = buf.length;
            buffers[n_buffers].start =
                    mmap(NULL /* start anywhere */,
                         buf.length,
                         PROT_READ | PROT_WRITE /* required */,
                         MAP_SHARED /* recommended */,
                         fd, buf.m.offset);

            if (MAP_FAILED == buffers[n_buffers].start)
                errno_exit("mmap");
        }

    }

    void Camera::init_device(void) {
        struct v4l2_capability cap;
        struct v4l2_format fmt;

        if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                fprintf(stderr, "%s is no V4L2 device\n", dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_QUERYCAP");
            }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "%s is no video capture device\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            fprintf(stderr, "%s does not support streaming i/o\n",
                    dev_name);
            exit(EXIT_FAILURE);
        }

        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width = 1280;
        fmt.fmt.pix.height = 800;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV21;
        fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

        if (-1 == ioctl(fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");

        init_mmap();

#ifdef ISP_FUNCTION
        //INIT RKISP
        _RKIspFunc.cam_ra_handle = dlopen("/usr/lib/libcam_ia.so", RTLD_LAZY | RTLD_GLOBAL);
        if (_RKIspFunc.cam_ra_handle == NULL) {
            mipicamera_debug ("open /usr/lib/libcam_ia.so failed, error %s\n", dlerror());
        }
        _RKIspFunc.rkisp_engine_handle = dlopen("/usr/lib/libcam_engine_cifisp.so", RTLD_LAZY | RTLD_GLOBAL);
        if (_RKIspFunc.rkisp_engine_handle == NULL) {
            mipicamera_debug ("open user-defined lib(%s) failed, reason:%s", "/usr/lib/libcam_engine_cifisp.so",
                              dlerror());
        } else {
            mipicamera_debug ("Load libcam_engine_cifisp.so successed\n");
            _RKIspFunc.start_func = (rkisp_start_func) dlsym(_RKIspFunc.rkisp_engine_handle, "rkisp_start");
            _RKIspFunc.stop_func = (rkisp_stop_func) dlsym(_RKIspFunc.rkisp_engine_handle, "rkisp_stop");
            if (_RKIspFunc.start_func == NULL) {
                mipicamera_debug ("func rkisp_start not found.");
                const char *errmsg;
                if ((errmsg = dlerror()) != NULL) {
                    mipicamera_debug("dlsym rkisp_start fail errmsg: %s", errmsg);
                }
            } else {
                mipicamera_debug("dlsym rkisp_start success\n");
            }
        }
#endif
    }


    void Camera::start_capturing(void) {
        unsigned int i;
        enum v4l2_buf_type type;

#ifdef ISP_FUNCTION
        if (_RKIspFunc.start_func != NULL) {
            printf("device manager start, capture dev fd: %d\n", fd);
            _RKIspFunc.start_func(_rkisp_engine, fd, isp_name, IQ_FILE, ctrl_mode);
            printf("device manager isp_init\n");

            if (_rkisp_engine == NULL) {
                printf("rkisp_init engine failed\n");
            } else {
                printf("rkisp_init engine succeed\n");
            }
        }
#endif
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                errno_exit("VIDIOC_QBUF");
        }
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
            errno_exit("VIDIOC_STREAMON");

    }

    void Camera::close_camera(void) {
        enum v4l2_buf_type type;

#ifdef ISP_FUNCTION
        //stop capture
        if (_RKIspFunc.stop_func != NULL) {
            printf("deinit rkisp engine\n");
            _RKIspFunc.stop_func(_rkisp_engine);
            dlclose(_RKIspFunc.cam_ra_handle);
            dlclose(_RKIspFunc.rkisp_engine_handle);
        }
#endif
        printf("Call VIDIOC_STREAMOFF\n");
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
            errno_exit("VIDIOC_STREAMOFF");

        //free buffer
        for (int i = 0; i < n_buffers; ++i)
            if (-1 == munmap(buffers[i].start, buffers[i].length))
                errno_exit("munmap");

        free(buffers);

        //close fd
        if (-1 == close(fd))
            errno_exit("close");

        fd = -1;
    }

    bool Camera::open_device(void) {
        fd = open(dev_name, O_RDWR /* required */ /*| O_NONBLOCK*/, 0);

        if (-1 == fd) {
            fprintf(stderr, "Cannot open '%s': %d, %s\n",
                    dev_name, errno, strerror(errno));
            return false;
        }
        return true;
    }

    bool Camera::open_camera(int seq) {
        if (seq == 0) {
            strcpy(dev_name, CAMERA1);
            strcpy(isp_name, ISPDEV1);
            ctrl_mode = CAMISP_CTRL_MASTER;
        } else if (seq == 1) {
            strcpy(dev_name, CAMERA2);
            strcpy(isp_name, ISPDEV2);
            ctrl_mode = CAMISP_CTRL_SLAVE;
        }

        if (!open_device()) {
            printf("%s fail\n", __func__);
            return false;
        }
        init_device();
        start_capturing();
    }

    //for stereo cameras
    bool Camera::throw_some_frames(int frames_count) {
        bool is_success = false;
        cv::Mat img;
        timeval time_val;
        while (frames_count--) {
            is_success = read_frame(&time_val, img);
        }

        if (!is_success) {
            printf("init fail for starting some frames\n");
            return false;
        } else {
            return true;
        }
    }

    bool Camera::synctime(timeval *time_ok, timeval *time_val, Mat &img) {
        int try_count = 0;
        double diff_time = mipi::frame_diff(time_val, time_ok);
        while (diff_time > kTPF && try_count++ < 5) {
            read_frame(time_val, img);
            diff_time = mipi::frame_diff(time_val, time_ok);
        }
        if (std::abs(diff_time) > kTPF)
            return false;
        return true;
    }
}
