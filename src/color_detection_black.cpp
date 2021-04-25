/*
  file: color_detection_black.cpp
  brief: detect black color in an image continuously (stereo cameras: kinect/realsense)
  details: opencv with HSV image detects black pixels and displays in white
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>


static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

int iLowH = 0;
int iHighH = 0;

int iLowS = 0; 
int iHighS = 0;

int iLowV = 0;
int iHighV = 0;

vector<int> pixel_x; // x of zip-tie
vector<int> pixel_y; // y of zip-tie

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_sub_;
  image_transport::Publisher image_pub_;
 

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/multisense/left/image_rect_color", 1, &ImageConverter::imageCb, this);
    //image_sub_ = it_.subscribe("/hsrb/head_rgbd_sensor/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    depth_sub_ = it_.subscribe("/multisense/depth", 1, &ImageConverter::depthCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(300);

    // // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());

    ////////////////////////////////////////////////////////////////////// color detection

    Mat imgHSV;

    cv::cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;

    // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    // inRange(imgHSV, Scalar(44,50,58,0), Scalar(46,52,60,0), imgThresholded);
    inRange(imgHSV, Scalar(0,0,0,0), Scalar(180,255,33.16,0), imgThresholded);  //black
    //morphological opening (remove small objects from the foreground)
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    // dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    // //morphological closing (fill small holes in the foreground)
    // dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    cout<<imgThresholded.size << endl; // 1080*1920 hd
    // cout << imgThresholded.at<bool>(0,0); //0-black, 255-white
    // cout << imgThresholded.size;
    // for (int i=0; i<539; i++) {
    //   for (int j=0; j<959; j++){
    //     cout << "i:" << i<< "////" << "j:" << j << "////"<< "value:" << imgThresholded.at<bool>(i,j) << "," << endl;
    //     if (imgThresholded.at<bool>(i,j) == 255) {
    //       pixel_x.push_back(i);
    //       pixel_y.push_back(j);
    //     }

    //   }
    // }

    cv::imshow("Thresholded Image", imgThresholded); //show the thresholded image
    cv::imshow("Original", cv_ptr->image); //show the original image

    if (waitKey(300) == 17) //wait for 'esc' key press for 300ms. If 'esc' key is pressed, break loop
       {
            cout << "esc key is pressed by user" << endl;
            exit(1);
       }
	

  }
  void depthCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr depth_img_cv;
    try
    {
    depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //cv::imshow("Depth", depth_img_cv->image); //show the depth image

  } 


 };


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
