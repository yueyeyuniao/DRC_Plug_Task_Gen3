#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include <image_geometry/pinhole_camera_model.h>
#include <vector>


static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber info_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
 
  image_geometry::PinholeCameraModel pin_hole_camera;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    // info_sub_ = nh_.subscribe("/kinect2/qhd/camera_info", 10, &ImageConverter::info_callback, this);
    // image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &ImageConverter::imageCb, this);

    info_sub_ = nh_.subscribe("/multisense/left/image_rect_color/camera_info", 10, &ImageConverter::info_callback, this);
    image_sub_ = it_.subscribe("/multisense/left/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    pin_hole_camera.fromCameraInfo(msg);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
// line segment
    // Mat Gray;
    // Mat image_canny;
    // cv::cvtColor(cv_ptr->image, Gray, CV_BGR2GRAY);  // CV_BGR2GRAY

    // Canny(cv_ptr->image, image_canny, 50, 200, 3);
    // cv::imshow(OPENCV_WINDOW, image_canny);
    // Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
    // double start = double(getTickCount());
    // vector<Vec4f> lines_std;
    // // Detect the lines
    // ls->detect(image_canny, lines_std);
    // double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
    // std::cout << "It took " << duration_ms << " ms." << std::endl;
    // // Show found lines
    // Mat drawnLines(image_canny);
    // ls->drawSegments(drawnLines, lines_std);
    // cv::imshow("Standard refinement", drawnLines);
    // cv::waitKey(300);
    // image_pub_.publish(cv_ptr->toImageMsg());

// hough circle detection
    Mat circle_gray;
    cv::cvtColor(cv_ptr->image, circle_gray, CV_BGR2GRAY); // convert image to gray
    GaussianBlur(circle_gray, circle_gray, Size(9, 9), 2, 2 ); // Reduce the noise so we avoid false circle detection
    vector<Vec3f> circles;
    HoughCircles(circle_gray, circles, CV_HOUGH_GRADIENT, 1, circle_gray.rows/100, 100, 28, 0, 40 );// Apply the Hough Transform to find the circles
    for(size_t i = 0; i < circles.size(); i++)  // Draw the circles
    {
      Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);
      // circle center
      circle(cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0);
      // circle outline
      circle(cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0);
    } 
    cv::imshow("Hough Circle", cv_ptr->image);
    cv::waitKey(300);
    image_pub_.publish(cv_ptr->toImageMsg());


  }
 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
