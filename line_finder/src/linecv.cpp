#include "line_finder/linecv.hpp"

#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using std::placeholders::_1;

namespace linecv {

LineCv::LineCv()
    : Node("linecv_node"),
      initial_persistence_count_(3),
      max_persistence_count_(5) {
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
  qos.avoid_ros_namespace_conventions(false);

  for (size_t i = 0; i < kMAP_PIXELS; i++) {
    for (size_t j = 0; j < kMAP_PIXELS; j++) {
      points_[i][j] = 0;
      if (i == (kMAP_PIXELS / 2)) {
        points_[i][j] = 255;
      }
    }
  }

  // cv::Mat mat = cv::Mat(3, 10, CV_8UC1);
  cv::Mat mat = cv::Mat(kMAP_PIXELS, kMAP_PIXELS, CV_8UC1);

  // The first element of temp decays to int* data.
  // It means that mat.data tracks temp.
  mat.data = points_[0];

  std::cout << "rows: " << mat.rows << ", cols: " << mat.cols << std::endl;
  // cv::namedWindow("scan", cv::WINDOW_NORMAL);
  // cv::resizeWindow("scan", 640, 640);
  // cv::imshow("scan", mat);
  // cv::waitKey();

  scan_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&LineCv::laserScanCallback, this, _1));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

LineCv::~LineCv() {}

// Compute the dot product AB . BC
float DotProduct(float pointA[], float pointB[], float pointC[]) {
  float AB[2];
  float BC[2];
  AB[0] = pointB[0] - pointA[0];
  AB[1] = pointB[1] - pointA[1];
  BC[0] = pointC[0] - pointB[0];
  BC[1] = pointC[1] - pointB[1];
  float dot = AB[0] * BC[0] + AB[1] * BC[1];

  return dot;
}

// Compute the cross product AB x AC
float CrossProduct(float pointA[], float pointB[], float pointC[]) {
  float AB[2];
  float AC[2];
  AB[0] = pointB[0] - pointA[0];
  AB[1] = pointB[1] - pointA[1];
  AC[0] = pointC[0] - pointA[0];
  AC[1] = pointC[1] - pointA[1];
  float cross = AB[0] * AC[1] - AB[1] * AC[0];

  return cross;
}

// Compute the distance from A to B
float Distance(float pointA[], float pointB[]) {
  float d1 = pointA[0] - pointB[0];
  float d2 = pointA[1] - pointB[1];

  return sqrt(d1 * d1 + d2 * d2);
}

// Compute the distance from AB to C
float LineToPointDistance2D(float pointA[], float pointB[], float pointC[]) {
  float dist = CrossProduct(pointA, pointB, pointC) / Distance(pointA, pointB);
  float dot1 = DotProduct(pointA, pointB, pointC);
  if (dot1 > 0) return Distance(pointB, pointC);

  float dot2 = DotProduct(pointB, pointA, pointC);
  if (dot2 > 0) return Distance(pointA, pointC);
  return abs(dist);
}

void LineCv::laserScanCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  RCLCPP_INFO(rclcpp::get_logger("linecv_node"), "scan");

  manage_scan(scan_msg);

  cv::Mat mat = cv::Mat(kMAP_PIXELS, kMAP_PIXELS, CV_8UC1);
  cv::Mat lineMat = cv::Mat(kMAP_PIXELS, kMAP_PIXELS, CV_8UC1);
  lineMat = cv::Scalar(0, 0, 0);

  for (size_t x_coord = 0; x_coord < kMAP_PIXELS; x_coord++) {
    for (size_t y_coord = 0; y_coord < kMAP_PIXELS; y_coord++) {
      mat.at<unsigned char>(x_coord, y_coord) =
          points_[x_coord][y_coord] > 0 ? 255 : 0;
    }
  }

  // cv::Mat dst = cv::Mat(kMAP_PIXELS, kMAP_PIXELS, CV_8UC1);

  // Probabilistic Line Transform
  std::vector<cv::Vec4i> linesP;  // will hold the results of the detection
  HoughLinesP(
      mat, linesP, 0.5 /*rho*/, CV_PI / 180 /*theta*/, 5 /*threshold*/,
      0.2 * kPIXELS_PER_METER /*minLineLength*/,
      0.5 * kPIXELS_PER_METER /*maxLineGap*/);  // runs the actual detection
  // Draw the lines
  size_t max_lines = linesP.size() > 100 ? 100 : linesP.size();
  // std::sort(linesP.begin(), linesP.end(),
  //      [](const std::vector<cv::Point>& c1, const std::vector<cv::Point>& c2)
  //      {
  //   return c1.x < c2.x;
  //      });
  struct sort_by_x_coord {
    bool operator()(cv::Vec4i const& a, cv::Vec4i const& b) const {
      if (a[0] < b[0]) return true;
      return false;
    }
  };
  // std::sort(linesP.begin(), linesP.end(), sort_by_y_coord());
  for (size_t i = 0; i < max_lines; i++) {
    cv::Vec4i l = linesP[i];
    float len = sqrt(((l[2] - l[0]) * (l[2] - l[0]) * 1.0) +
                     ((l[3] - l[1]) * (l[3] - l[1]) * 1.0)) /
                kPIXELS_PER_METER;
    float v[] = {l[0] * 1.0f, l[1] * 1.0f};
    float w[] = {l[2] * 1.0f, l[3] * 1.0f};
    float origin[] = {kPIXELS_PER_METER * kMAX_RANGE * 1.0f, kPIXELS_PER_METER * kMAX_RANGE * 1.0f};
    float dist_from_origin =  LineToPointDistance2D(v, w, origin) / kPIXELS_PER_METER;
    std::cout << "[" << i << "/" << linesP.size() << "] (" << l[0] << ","
              << l[1] << ")->(" << l[2] << "," << l[3] << "), len: " << len
              << "(" << l[0] * 1.0f / kPIXELS_PER_METER << ","
              << l[1] * 1.0f / kPIXELS_PER_METER << ")->(" << l[2] * 1.0f / kPIXELS_PER_METER
              << ", " << l[3] * 1.0f / kPIXELS_PER_METER << ")"
              << ", distToLine: " << dist_from_origin
               << std::endl;
    line(lineMat, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
         cv::Scalar(255, 255, 255), 2 /*thickness*/, cv::LINE_AA);
    cv::putText(lineMat, std::to_string(i), cv::Point(l[0], l[1]),
                cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255), 2);
  }

  cv::namedWindow("scan", cv::WINDOW_NORMAL);
  cv::resizeWindow("scan", 640, 640);
  cv::imshow("scan", mat);

  cv::namedWindow("lines", cv::WINDOW_NORMAL);
  cv::resizeWindow("lines", 640, 640);
  cv::imshow("lines", lineMat);
  cv::waitKey(/*1*/);

  // for (size_t x_coord = 0; x_coord < kMAP_PIXELS; x_coord++) {
  //   for (size_t y_coord = 0; y_coord < kMAP_PIXELS; y_coord++) {
  //   }
  // }

  // for (size_t x_coord = 0; x_coord < kMAP_PIXELS; x_coord++) {
  //   for (size_t y_coord = 0; y_coord < kMAP_PIXELS; y_coord++) {
  //     float offset = kMAP_PIXELS / 2.0;
  //     float px = x_coord - offset;  // Offset to represent real coordinates.
  //     float py = y_coord - offset;
  //     px = px / kPIXELS_PER_METER;  // Scale.
  //     py = py / kPIXELS_PER_METER;
  //     float distance_to_point = sqrt((px * px) + (py * py));
  //     float angle_to_point = atan2(py, px);
  //     int angle_slot =
  //         (angle_to_point - scan_msg->angle_min) / scan_msg->angle_increment;
  //     if (points_[x_coord][y_coord] > 0) {
  //       // Found a persistent point.
  //       // if (distance_to_point < new_scan.ranges[angle_slot]) {
  //       // new_scan.ranges[angle_slot] = distance_to_point;
  //       // }
  //     }
  //   }
  // }
}

void LineCv::manage_scan(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
  static bool initialized = false;
  if (!initialized) {
    for (size_t i = 0; i < kMAP_PIXELS; i++) {
      for (size_t j = 0; j < kMAP_PIXELS; j++) {
        points_[i][j] = 0;
      }
    }

    initialized = true;
  }

  try {
    tf2::TimePoint time_point = tf2_ros::fromMsg(scan_msg->header.stamp);
    bool canTransform =
        tf_buffer_->canTransform("odom", scan_msg->header.frame_id.c_str(),
                                 time_point, tf2::durationFromSec(1.0));
    if (!canTransform) {
      RCLCPP_WARN(get_logger(),
                  "Could not look up transform from %s to %s at time [%d.%d]",
                  scan_msg->header.frame_id.c_str(), "odom",
                  scan_msg->header.stamp.sec, scan_msg->header.stamp.nanosec);
    } else {
      for (size_t i = 0; i < kMAP_PIXELS; i++) {
        for (size_t j = 0; j < kMAP_PIXELS; j++) {
          if (points_[i][j] > 0) {
            points_[i][j] -= 1;
          }
        }
      }

      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer_->lookupTransform("odom", scan_msg->header.frame_id,
                                      tf2::TimePointZero);
      float angle = scan_msg->angle_min;
      for (float range : scan_msg->ranges) {
        if ((range < (float)kMAX_RANGE) && (range > -(float)kMAX_RANGE)) {
          geometry_msgs::msg::PointStamped in, out;
          in.header = scan_msg->header;
          in.point.x = range * cos(angle);
          in.point.y = range * sin(angle);
          in.point.z = 0;
          tf_buffer_->transform(in, out, "odom");
          int x_coord = (out.point.x * kPIXELS_PER_METER) + (kMAP_PIXELS / 2);
          int y_coord = (out.point.y * kPIXELS_PER_METER) + (kMAP_PIXELS / 2);
          if ((x_coord >= 0) && (y_coord >= 0) &&
              ((uint32_t)x_coord < kMAP_PIXELS) &&
              ((uint32_t)y_coord < kMAP_PIXELS) &&
              (points_[x_coord][y_coord] < max_persistence_count_)) {
            points_[x_coord][y_coord] += initial_persistence_count_;
          }
        }
        angle += scan_msg->angle_increment;
      }
    }
  } catch (tf2::TransformException& ex) {
    std::cout << "Exception: " << ex.what() << std::endl;
  }
}

uint8_t LineCv::points_[kMAP_PIXELS][kMAP_PIXELS];

}  // namespace linecv

// #include "opencv2/highgui.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc.hpp"

// using namespace cv;
// using namespace std;

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);

//   // Declare the output variables
//   Mat dst, cdst, cdstP;
//   const char* default_file = "sudoku.png";
//   const char* filename = argc >= 2 ? argv[1] : default_file;
//   // Loads an image
//   Mat src = imread(samples::findFile(filename), IMREAD_GRAYSCALE);
//   // Check if image is loaded fine
//   if (src.empty()) {
//     printf(" Error opening image\n");
//     printf(" Program Arguments: [image_name -- default %s] \n",
//     default_file); return -1;
//   }
//   // Edge detection
//   Canny(src, dst, 50, 200, 3);
//   // Copy edges to the images that will display the results in BGR
//   cvtColor(dst, cdst, COLOR_GRAY2BGR);
//   cdstP = cdst.clone();
//   // Standard Hough Line Transform
//   vector<Vec2f> lines;  // will hold the results of the detection
//   HoughLines(dst, lines, 1, CV_PI / 180, 150, 0,
//              0);  // runs the actual detection
//   // Draw the lines
//   for (size_t i = 0; i < lines.size(); i++) {
//     float rho = lines[i][0], theta = lines[i][1];
//     Point pt1, pt2;
//     double a = cos(theta), b = sin(theta);
//     double x0 = a * rho, y0 = b * rho;
//     pt1.x = cvRound(x0 + 1000 * (-b));
//     pt1.y = cvRound(y0 + 1000 * (a));
//     pt2.x = cvRound(x0 - 1000 * (-b));
//     pt2.y = cvRound(y0 - 1000 * (a));
//     line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
//   }
//   // Probabilistic Line Transform
//   vector<Vec4i> linesP;  // will hold the results of the detection
//   HoughLinesP(dst, linesP, 1, CV_PI / 180, 50, 50,
//               10);  // runs the actual detection
//   // Draw the lines
//   for (size_t i = 0; i < linesP.size(); i++) {
//     Vec4i l = linesP[i];
//     line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3,
//          LINE_AA);
//   }
//   // Show results
//   imshow("Source", src);
//   imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
//   imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
//   // Wait and Exit
//   // waitKey();

//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }