//
// Created by chary on 18-10-22.
//

#ifndef MIPIWRAPPER_MIPI_CAMERA_HPP
#define MIPIWRAPPER_MIPI_CAMERA_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rkisp_interface.h"

namespace mipi {

    const double kMicroScale = 0.000001;
    const double kTPF = 1.0 / 30.0 * 0.5;

    double frame_diff(timeval *time_val0, timeval *time_val1); 

    typedef int (*rkisp_start_func)(void *&engine, int vidFd, const char *ispNode, const char *tuningFile,
                                    enum CAMISP_CTRL_MODE ctrl_mode);

    typedef int (*rkisp_stop_func)(void *&engine);

    struct RKIspFunc {
        void *cam_ra_handle;
        void *rkisp_engine_handle;
        rkisp_start_func start_func;
        rkisp_stop_func stop_func;
    };

    struct buffer {
        void *start;
        size_t length;
    };

#define CAMERA1    "/dev/video0"
#define ISPDEV1    "/dev/video1"
#define CAMERA2    "/dev/video4"
#define ISPDEV2    "/dev/video5"
#define IQ_FILE  "/etc/cam_iq.xml"

#define ISP_FUNCTION

#define LOG_PRINT

#ifdef LOG_PRINT
#define mipicamera_debug(fmt, args...) printf(fmt, ## args)
#else
#define mipicamera_debug(fmt, args...)
#endif

    class Camera {

    private:
        int fd;
        char dev_name[255];
        char isp_name[255];
        unsigned int n_buffers;
        struct buffer *buffers;
        void *_rkisp_engine;
        enum CAMISP_CTRL_MODE ctrl_mode;
        struct RKIspFunc _RKIspFunc;

        void init_device(void);

        void init_mmap(void);

        bool open_device(void);

        void start_capturing(void);

    public:
        ~Camera();

        bool open_camera(int seq);

        bool read_frame(timeval *time_val, cv::Mat &img);

        void close_camera(void);

        bool throw_some_frames(int frames_count);

        bool synctime(timeval *time_ok, timeval *time_val, cv::Mat &img);
    };
}

#endif //MIPIWRAPPER_MIPI_CAMERA_HPP
