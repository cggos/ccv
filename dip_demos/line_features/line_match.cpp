#include <iostream>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_FEATURES2D

#include <opencv2/line_descriptor.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#define MATCHES_DIST_THRESHOLD 25

using namespace cv;
using namespace cv::line_descriptor;

static const char* keys =
        { "{@image_path1 | | Image path 1 }"
          "{@image_path2 | | Image path 2 }" };

static void help()
{
    std::cout << "\nThis example shows the functionalities of lines extraction " << "and descriptors computation furnished by BinaryDescriptor class\n"
              << "Please, run this sample using a command in the form\n" << "./example_line_descriptor_compute_descriptors <path_to_input_image 1>"
              << "<path_to_input_image 2>" << std::endl;

}

int main( int argc, char** argv ) {
    CommandLineParser parser(argc, argv, keys);

    String image_path1 = parser.get<String>(0);
    String image_path2 = parser.get<String>(1);

    if (image_path1.empty() || image_path2.empty()) {
        help();
        return -1;
    }

    cv::Mat imageMat1 = imread(image_path1, 1);
    cv::Mat imageMat2 = imread(image_path2, 1);

    if (imageMat1.data == NULL || imageMat2.data == NULL) {
        std::cout << "Error, images could not be loaded. Please, check their path" << std::endl;
    }

    cv::Mat mask1 = Mat::ones(imageMat1.size(), CV_8UC1);
    cv::Mat mask2 = Mat::ones(imageMat2.size(), CV_8UC1);

    std::vector<KeyLine> keylines1, keylines2;
    cv::Mat descr1, descr2;

    Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();

    (*bd)(imageMat1, mask1, keylines1, descr1, false, false);
    (*bd)(imageMat2, mask2, keylines2, descr2, false, false);

    /* select keylines from first octave and their descriptors */
    std::vector<KeyLine> lbd_octave1, lbd_octave2;
    Mat left_lbd, right_lbd;
    for (int i = 0; i < (int) keylines1.size(); i++) {
        if (keylines1[i].octave == 0) {
            lbd_octave1.push_back(keylines1[i]);
            left_lbd.push_back(descr1.row(i));
        }
    }
    for (int j = 0; j < (int) keylines2.size(); j++) {
        if (keylines2[j].octave == 0) {
            lbd_octave2.push_back(keylines2[j]);
            right_lbd.push_back(descr2.row(j));
        }
    }

    std::vector<DMatch> matches;
    Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
    bdm->match(left_lbd, right_lbd, matches);

    std::vector<DMatch> good_matches;
    for (int i = 0; i < (int) matches.size(); i++) {
        if (matches[i].distance < MATCHES_DIST_THRESHOLD)
            good_matches.push_back(matches[i]);
    }

    cv::Mat outImg;
    cv::Mat scaled1, scaled2;
    std::vector<char> mask(matches.size(), 1);
    drawLineMatches(imageMat1, lbd_octave1, imageMat2, lbd_octave2, good_matches, outImg, Scalar::all(-1),
                    Scalar::all(-1), mask,
                    DrawLinesMatchesFlags::DEFAULT);

    imshow("Matches", outImg);
    waitKey();
    imwrite("/home/ubisum/Desktop/images/env_match/matches.jpg", outImg);

    Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();

    std::vector<KeyLine> klsd1, klsd2;
    Mat lsd_descr1, lsd_descr2;
    lsd->detect(imageMat1, klsd1, 2, 2, mask1);
    lsd->detect(imageMat2, klsd2, 2, 2, mask2);

    /* compute descriptors for lines from first octave */
    bd->compute(imageMat1, klsd1, lsd_descr1);
    bd->compute(imageMat2, klsd2, lsd_descr2);

    std::vector<KeyLine> octave0_1, octave0_2;
    Mat leftDEscr, rightDescr;
    for (int i = 0; i < (int) klsd1.size(); i++) {
        if (klsd1[i].octave == 1) {
            octave0_1.push_back(klsd1[i]);
            leftDEscr.push_back(lsd_descr1.row(i));
        }
    }
    for (int j = 0; j < (int) klsd2.size(); j++) {
        if (klsd2[j].octave == 1) {
            octave0_2.push_back(klsd2[j]);
            rightDescr.push_back(lsd_descr2.row(j));
        }
    }

    std::vector<DMatch> lsd_matches;
    bdm->match(leftDEscr, rightDescr, lsd_matches);
    good_matches.clear();
    for (int i = 0; i < (int) lsd_matches.size(); i++) {
        if (lsd_matches[i].distance < MATCHES_DIST_THRESHOLD)
            good_matches.push_back(lsd_matches[i]);
    }

    cv::Mat lsd_outImg;
    resize(imageMat1, imageMat1, Size(imageMat1.cols / 2, imageMat1.rows / 2));
    resize(imageMat2, imageMat2, Size(imageMat2.cols / 2, imageMat2.rows / 2));
    std::vector<char> lsd_mask(matches.size(), 1);
    drawLineMatches(imageMat1, octave0_1, imageMat2, octave0_2, good_matches, lsd_outImg, Scalar::all(-1),
                    Scalar::all(-1), lsd_mask,
                    DrawLinesMatchesFlags::DEFAULT);

    imshow("LSD matches", lsd_outImg);
    waitKey();
}

#else

int main()
 {
     std::cerr << "OpenCV was built without features2d module" << std::endl;
     return 0;
 }

#endif // HAVE_OPENCV_FEATURES2D