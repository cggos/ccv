#ifndef CAM_V4L2_H_
#define CAM_V4L2_H_

#include <string>

#include <linux/videodev2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace cg {

    class CamV4l2 {

    public:
        CamV4l2() {}

        CamV4l2(std::string device_name, int img_w, int img_h) :
                device_name_(device_name), img_w_(img_w), img_h_(img_h), n_req_buf_(1) {
            if(init()<0) {
                throw std::runtime_error("ERROR: CamV4l2 init");
            }
        }

        ~CamV4l2() { free(); }

        cv::Mat read();

        int save(std::string file_name);

    private:
        int init();
        int free();

    private:
        std::string device_name_;
        int fd_video_;
        int img_w_;
        int img_h_;

        struct buffer {
            void *start;
            unsigned int length;
        }*buffers_;

        struct v4l2_buffer buf_;
        int n_req_buf_;

        cv::Mat mat_src_;
        cv::Mat mat_dst_;
    };
}

#endif // CAM_V4L2_H_