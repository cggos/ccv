#include <algorithm>
#include <iostream>
#include <limits>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

cv::Mat DetectEdges(const cv::Mat& image, double low_threshold, double high_threshold) {
  cv::Mat gray;
  if (image.channels() == 1) {
    gray = image;
  } else {
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
  }

  cv::Mat smoothed;
  cv::GaussianBlur(gray, smoothed, cv::Size(5, 5), 1.0);

  cv::Mat edges;
  cv::Canny(smoothed, edges, low_threshold, high_threshold);
  return edges;
}

struct MatchResult {
  cv::Point location;
  double score = std::numeric_limits<double>::infinity();
};

MatchResult ChamferMatch(const cv::Mat& template_edges, const cv::Mat& search_edges) {
  std::vector<cv::Point> template_points;
  cv::findNonZero(template_edges, template_points);
  if (template_points.empty()) {
    throw std::runtime_error("no edge was found in the template image");
  }

  // distanceTransform calculates the distance to the nearest zero pixel. Make
  // search-image edges zero so every pixel stores its distance to an edge.
  cv::Mat inverted_edges;
  cv::bitwise_not(search_edges, inverted_edges);
  cv::Mat distance_map;
  cv::distanceTransform(inverted_edges, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  MatchResult best;
  const int max_x = search_edges.cols - template_edges.cols;
  const int max_y = search_edges.rows - template_edges.rows;
  for (int y = 0; y <= max_y; ++y) {
    for (int x = 0; x <= max_x; ++x) {
      double distance_sum = 0.0;
      for (const cv::Point& point : template_points) {
        distance_sum += distance_map.at<float>(y + point.y, x + point.x);
      }

      const double score = distance_sum / static_cast<double>(template_points.size());
      if (score < best.score) {
        best.location = cv::Point(x, y);
        best.score = score;
      }
    }
  }
  return best;
}

void PrintUsage(const char* program) {
  std::cerr << "Usage: " << program << " <template_image> <search_image> [output_image] [canny_low] [canny_high]\n"
            << "Example: " << program << " template.png scene.png chamfer_result.png 50 150\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argc > 6) {
    PrintUsage(argv[0]);
    return 1;
  }

  const std::string output_path = argc >= 4 ? argv[3] : "chamfer_result.png";
  double canny_low = 50.0;
  double canny_high = 150.0;
  try {
    canny_low = argc >= 5 ? std::stod(argv[4]) : canny_low;
    canny_high = argc >= 6 ? std::stod(argv[5]) : canny_high;
  } catch (const std::exception&) {
    std::cerr << "Canny thresholds must be numbers.\n";
    return 1;
  }
  if (canny_low < 0.0 || canny_high <= canny_low) {
    std::cerr << "Canny thresholds must satisfy 0 <= low < high.\n";
    return 1;
  }

  const cv::Mat template_image = cv::imread(argv[1], cv::IMREAD_COLOR);
  const cv::Mat search_image = cv::imread(argv[2], cv::IMREAD_COLOR);
  if (template_image.empty() || search_image.empty()) {
    std::cerr << "Failed to read the input images.\n";
    return 1;
  }
  if (template_image.cols > search_image.cols || template_image.rows > search_image.rows) {
    std::cerr << "The template image must not be larger than the search image.\n";
    return 1;
  }

  try {
    const cv::Mat template_edges = DetectEdges(template_image, canny_low, canny_high);
    const cv::Mat search_edges = DetectEdges(search_image, canny_low, canny_high);
    const MatchResult match = ChamferMatch(template_edges, search_edges);

    cv::Mat visualization = search_image.clone();
    const cv::Rect match_box(match.location, template_image.size());
    cv::rectangle(visualization, match_box, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    cv::Mat colored_edges;
    cv::cvtColor(search_edges, colored_edges, cv::COLOR_GRAY2BGR);
    colored_edges.copyTo(visualization, search_edges);
    cv::rectangle(visualization, match_box, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);

    if (!cv::imwrite(output_path, visualization)) {
      std::cerr << "Failed to write result image: " << output_path << '\n';
      return 1;
    }

    std::cout << "best_location=" << match.location.x << ',' << match.location.y << '\n'
              << "chamfer_score=" << match.score << '\n'
              << "result=" << output_path << '\n';
  } catch (const cv::Exception& error) {
    std::cerr << "OpenCV error: " << error.what() << '\n';
    return 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }

  return 0;
}
