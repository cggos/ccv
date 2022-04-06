#ifndef MYNTEYE_API_H
#define MYNTEYE_API_H

#include <linux/videodev2.h>

#include <iostream>
#include <opencv2/videoio/videoio.hpp>
#include <stdexcept>

class MyntEyeCam {
 public:
  enum CAM_TYPE { S1030 = 1000, D1000 };

  MyntEyeCam(const int device_idx, const CAM_TYPE cam_type = S1030) : cam_type_(cam_type) {
    try {
      cap_.open(device_idx);
      if (!cap_.isOpened()) {
        throw std::runtime_error("ERROR: Open Cam Failed.\n");
      }
      set_cap();
    } catch (const std::exception& e) {
      std::cerr << e.what() << std::endl;
    }
  }

  ~MyntEyeCam() { cap_.release(); }

  inline bool is_opened() const { return cap_.isOpened(); }

  inline void set_cap() {
    switch (cam_type_) {
      case S1030:
        cap_.set(cv::CAP_PROP_CONVERT_RGB, 0);  // !!!
        break;
      case D1000:
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        // cap_depth.set(CAP_PROP_FRAME_WIDTH,1280);
        // cap_depth.set(CAP_PROP_FRAME_HEIGHT,720);
      default:
        break;
    }
  }

  inline void get_cap() const {
    unsigned int fmt = cap_.get(cv::CAP_PROP_FORMAT);
    unsigned int mode = cap_.get(cv::CAP_PROP_MODE);
    int w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    int h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    int fps = cap_.get(cv::CAP_PROP_FPS);

    std::string str_mode = "";
    switch (mode) {
      case V4L2_PIX_FMT_BGR24:
        str_mode = "FMT_BGR24";
        break;
      case V4L2_PIX_FMT_YVU420:
        str_mode = "FMT_YVU420";
        break;
      case V4L2_PIX_FMT_YUV411P:
        str_mode = "FMT_YUV411P";
        break;
      case V4L2_PIX_FMT_MJPEG:
        str_mode = "FMT_MJPG";
        break;
      case V4L2_PIX_FMT_JPEG:
        str_mode = "FMT_JPG";
        break;
      case V4L2_PIX_FMT_YUYV:
        str_mode = "FMT_YUYV";
        break;
      case V4L2_PIX_FMT_RGB24:
        str_mode = "FMT_RGB24";
        break;
      case V4L2_PIX_FMT_UYVY:
        str_mode = "FMT_UYVY";
        break;
      default:
        str_mode = "mode not found!";
        break;
    }

    std::cout << "cap format: " << fmt << std::endl;
    std::cout << "cap mode: " << mode << ", " << str_mode << std::endl;
    std::cout << "cap frame width:  " << w << std::endl;
    std::cout << "cap frame height: " << h << std::endl;
    std::cout << "cap frame: " << fps << std::endl;
  }

  void read() {
    cv::Mat raw;
    cap_.read(raw);

    switch (cam_type_) {
      case S1030: {
        img_l_ = cv::Mat(raw.size(), CV_8UC1);
        img_r_ = cv::Mat(raw.size(), CV_8UC1);
        for (int i = 0; i < raw.rows * raw.cols; ++i) {
          img_l_.data[i] = raw.data[i * 2];
          img_r_.data[i] = raw.data[i * 2 + 1];
        }
      } break;
      case D1000: {
        // TODO: test
        int w = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        int h = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        // color image(1280*720 * 2)
        //  left = Mat(h, w/2, CV_8UC3);
        //  right = Mat(h, w/2, CV_8UC3);
        const int w1 = 720;
        img_l_ = cv::Mat(h, w1, CV_8UC3);
        img_r_ = cv::Mat(h, w1, CV_8UC3);
        // gray image
        //  cvtColor(raw,raw,CV_BGR2GRAY);
        //  left = Mat(h,w/2,CV_8UC1);
        //  right = Mat(h,w/2,CV_8UC1);

        // use ptr
        //  //left frame
        //  for (int i = 0; i < h ; i++)
        //  {
        //      uchar *ptr = raw.ptr<uchar> (i);
        //      for( int j = 0; j < (w * 3) / 2; j++)
        //      left.ptr<uchar>(i) [j] = ptr[j];
        //  }

        // //right frame
        // for (int i = 0; i < h; i++)
        // {
        //     uchar *ptr = raw.ptr<uchar> (i);
        //     for( int j = (w * 3) /2 ; j < (w * 3); j++ )
        //     right.ptr<uchar>(i) [j] = ptr[j];
        // }

        // use ROI

        // use color image,only use width and height,don't need to consider channel
        img_l_ = raw(cv::Rect((w / 4 - w1 / 2), 0, (w / 4 + w1 / 2), h));
        img_r_ = raw(cv::Rect((3 * w / 4 - w1 / 2), 0, (3 * w / 4 + w1 / 2), h));

        // use gray image
        //  left = raw(Rect(0,0,w/2,h));
        //  right = raw(Rect(w/2,0,w/2 ,h));

        // Mat depth(h_d,w_d,CV_16UC1);
        // cap_depth.read(depth);
        // cout << depth <<endl;
        // imshow("depth",depth);
        // imshow("raw",raw);
      } break;
    }
  }

  cv::Mat get_img_l() const { return img_l_; }
  cv::Mat get_img_r() const { return img_r_; }

 private:
  cv::VideoCapture cap_;
  CAM_TYPE cam_type_;

  cv::Mat img_l_;
  cv::Mat img_r_;
};

#endif  // MYNTEYE_API_H