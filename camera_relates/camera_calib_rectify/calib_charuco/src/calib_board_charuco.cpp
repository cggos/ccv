#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

namespace {
const char* about =
        "Calibration using a ChArUco board\n"
        "  To capture a frame for calibration, press 'c',\n"
        "  If input comes from video, press any key for next frame\n"
        "  To finish capturing, press 'ESC' key and calibration starts.\n";
const char* keys  =
        "{w        |       | Number of squares in X direction }"
        "{h        |       | Number of squares in Y direction }"
        "{sl       |       | Square side length (in meters) }"
        "{ml       |       | Marker side length (in meters) }"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{@outfile |<none> | Output file with calibrated camera parameters }"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{dp       |       | File of marker detector parameters }"
        "{rs       |       | Apply refind strategy }"
        "{r        |       | show rejected candidates too }"
        "{zt       | false | Assume zero tangential distortion }"
        "{a        |       | Fix aspect ratio (fx/fy) to this value }"
        "{pc       | false | Fix the principal point at the center }"
        "{sc       | false | Show detected chessboard corners after calibration }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}


/**
 */
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}


/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if(argc < 6) {
        parser.printMessage();
        return 0;
    }

    int squaresX = parser.get<int>("w");
    int squaresY = parser.get<int>("h");
    float squareLength = parser.get<float>("sl");
    float markerLength = parser.get<float>("ml");
    int dictionaryId = parser.get<int>("d");
    bool showRejected = parser.has("r");
    bool refindStrategy = parser.has("rs");
    int camId = parser.get<int>("ci");

    bool showChessboardCorners = parser.get<bool>("sc");

    string outputFile = parser.get<string>(0);

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    int calibrationFlags = 0;
    float aspectRatio = 1;
    if(parser.has("a")) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if(parser.get<bool>("zt")) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    if(parser.get<bool>("pc")) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        return 0;
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    // create charuco board object
    Ptr<aruco::CharucoBoard> charucoboard = aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    // collect data from each frame
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    vector< Mat > allImgs;
    Size imgSize;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);

        vector< int > markerIds;
        vector< vector< Point2f > > markerCorners, rejectedMarkers;

        aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedMarkers);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers);

        {
            vector< int > charucoIds;
            vector< Point2f > charucoCorners;
            int interpolatedCorners = 0;
            if(markerIds.size() > 0)
                interpolatedCorners =
                        aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard, charucoCorners, charucoIds);
            if (interpolatedCorners > 0) {
                Scalar color;
                color = Scalar(255, 0, 0);
                aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
            }

            if (markerIds.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, markerCorners);
            if (showRejected && rejectedMarkers.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));
        }

        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        static int n_count = 0;
        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        if(key == 'c' && markerIds.size() > 0) {
            cout << "Frame captured " << n_count << endl;
            imwrite("imageCopy.bmp", image);
            allCorners.push_back(markerCorners);
            allIds.push_back(markerIds);
            allImgs.push_back(image);
            imgSize = image.size();
            n_count++;
        }
    }

    // for calib
    {
        if (allIds.size() < 1) {
            cerr << "Not enough captures for calibration" << endl;
            return 0;
        }

        cv::Mat cameraMatrix;
        if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
            cameraMatrix = Mat::eye(3, 3, CV_64F);
            cameraMatrix.at< double >(0, 0) = aspectRatio;
        }
        cv::Mat distCoeffs;
        vector< Mat > rvecs, tvecs;

#if 0
        // for aruco calibration
        {
            // prepare data for calibration
            vector<vector<Point2f> > allCornersConcatenated;
            vector<int> allIdsConcatenated;
            vector<int> markerCounterPerFrame;
            markerCounterPerFrame.reserve(allCorners.size());
            for (unsigned int i = 0; i < allCorners.size(); i++) {
                markerCounterPerFrame.push_back((int) allCorners[i].size());
                for (unsigned int j = 0; j < allCorners[i].size(); j++) {
                    allCornersConcatenated.push_back(allCorners[i][j]);
                    allIdsConcatenated.push_back(allIds[i][j]);
                }
            }

            // for each frame, get properly processed imagePoints and objectPoints for the calibrateCamera function
            size_t nFrames = markerCounterPerFrame.size();

            vector<vector<Point2f> > processedImagePoints;
            vector<vector<Point3f> > processedObjectPoints;

            int markerCounter = 0;
            for (size_t frame = 0; frame < nFrames; frame++) {
                int nMarkersInThisFrame = markerCounterPerFrame[frame];

                CV_Assert(nMarkersInThisFrame > 0);

                vector<vector<Point2f> > thisFrameCorners;
                vector<int> thisFrameIds;
                thisFrameCorners.reserve((size_t) nMarkersInThisFrame);
                thisFrameIds.reserve((size_t) nMarkersInThisFrame);
                for (int j = markerCounter; j < markerCounter + nMarkersInThisFrame; j++) {
                    thisFrameCorners.push_back(allCornersConcatenated[j]);
                    thisFrameIds.push_back(allIdsConcatenated[j]);
                }
                markerCounter += nMarkersInThisFrame;

                vector<Point2f> currentImgPoints;
                vector<Point3f> currentObjPoints;
                // Given a board configuration and a set of detected markers, returns the corresponding
                // image points and object points to call solvePnP
                aruco::getBoardObjectAndImagePoints(board, thisFrameCorners, thisFrameIds, currentObjPoints,
                                                    currentImgPoints);
                if (currentImgPoints.size() > 0 && currentObjPoints.size() > 0) {
                    processedImagePoints.push_back(currentImgPoints);
                    processedObjectPoints.push_back(currentObjPoints);
                }
            }

            // calibrate camera using aruco markers
            double arucoRepErr =
                    cv::calibrateCamera(
                            processedObjectPoints,
                            processedImagePoints,
                            imgSize,
                            cameraMatrix,
                            distCoeffs,
                            noArray(), noArray(),
                            calibrationFlags);

            cout << "Rep Error Aruco: " << arucoRepErr << endl;
        }
#endif


#if 1
        // for charuco calibration
        {
            // prepare data for charuco calibration
            int nFrames = (int) allCorners.size();
            vector<vector<Point2f> > allCharucoCorners;
            vector<vector<int> > allCharucoIds;
            vector<Mat> filteredImages;
            allCharucoCorners.reserve(nFrames);
            allCharucoIds.reserve(nFrames);

            for (int i = 0; i < nFrames; i++) {
                // interpolate using camera parameters
                vector<Point2f> currentCharucoCorners;
                vector<int> currentCharucoIds;
                aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImgs[i], charucoboard,
                                                 currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                                 distCoeffs);

                allCharucoCorners.push_back(currentCharucoCorners);
                allCharucoIds.push_back(currentCharucoIds);
                filteredImages.push_back(allImgs[i]);
            }

            if (allCharucoCorners.size() < 4) {
                cerr << "Not enough corners for calibration" << endl;
                return 0;
            }

            CV_Assert(allCharucoIds.size() > 0 && (allCharucoIds.size() == allCharucoCorners.size()));

            // Join object points of charuco corners in a single vector for calibrateCamera() function
            vector<vector<Point3f> > allObjPoints;
            allObjPoints.resize(allCharucoIds.size());
            for (unsigned int i = 0; i < allCharucoIds.size(); i++) {
                unsigned int nCorners = (unsigned int) allCharucoIds[i].size();
                CV_Assert(nCorners > 0 && nCorners == allCharucoCorners[i].size());
                allObjPoints[i].reserve(nCorners);

                for (unsigned int j = 0; j < nCorners; j++) {
                    int pointId = allCharucoIds[i][j];
                    CV_Assert(pointId >= 0 && pointId < (int) charucoboard->chessboardCorners.size());
                    allObjPoints[i].push_back(charucoboard->chessboardCorners[pointId]);
                }
            }

//            ofstream outfile("out.txt");
//            std::cout << "write file" << std::endl;
//            for(int i=0; i<allCharucoCorners[0].size(); ++i) {
//                outfile << allCharucoCorners[0][i] << "\t" << allObjPoints[0][i] << std::endl;
//            }
//            outfile.close();

            double repError =
                    calibrateCamera(
                            allObjPoints,
                            allCharucoCorners,
                            imgSize,
                            cameraMatrix,
                            distCoeffs,
                            rvecs, tvecs,
                            noArray(), noArray(), noArray(),
                            calibrationFlags);

            cout << "Rep Error: " << repError << endl;

            bool saveOk =  saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags,
                                            cameraMatrix, distCoeffs, repError);

            if(!saveOk) {
                cerr << "Cannot save output file" << endl;
                return 0;
            }
            cout << "Calibration saved to " << outputFile << endl;

            // show interpolated charuco corners for debugging
            if(showChessboardCorners) {
                for(unsigned int frame = 0; frame < filteredImages.size(); frame++) {
                    Mat imageCopy = filteredImages[frame].clone();
                    if(allIds[frame].size() > 0)
                        if(allCharucoCorners[frame].size() > 0)
                            aruco::drawDetectedCornersCharuco( imageCopy, allCharucoCorners[frame], allCharucoIds[frame]);

                    imshow("out", imageCopy);
                    char key = (char)waitKey(0);
                    if(key == 27) break;
                }
            }
        }
#endif

    }

    return 0;
}
