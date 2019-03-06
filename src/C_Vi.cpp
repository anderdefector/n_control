//http://docs.opencv.org/3.1.0
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>//imgproc
#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>
//#include "opencv2/video/tracking.hpp"
// Thresholds
int minH = 0, maxH = 116;//116
int minS = 0, maxS = 52;
int minV = 70, maxV = 186;
int mx=0;
int my=0;
class Edge_Detector
{
  ros::NodeHandle nh1_;
  image_transport::ImageTransport it1_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher mx_pub;
  ros::Publisher my_pub;
public:
  Edge_Detector()
    : it1_(nh1_)
  {
    // Subscribe to input video feed and publish output video feed
    //image_sub_ = it1_.subscribe("/bebop/image_raw", 50, &Edge_Detector::imageCb, this);
    image_sub_ = it1_.subscribe("cv_camera/image_raw", 50, &Edge_Detector::imageCb, this);	
    mx_pub = nh1_.advertise<std_msgs::Int32>("/mx_c",1);
    //r_pub = nh1_.advertise<std_msgs::Int32>("/radio",1);
    //cv::namedWindow(OPENCV_WINDOW);
//*********************************************************************
	// Create a window
	cv::namedWindow("HSV Parameters");
	cv::createTrackbar("H max", "HSV Parameters", &maxH, 255);
	cv::createTrackbar("H min", "HSV Parameters", &minH, 255);
	cv::createTrackbar("S max", "HSV Parameters", &maxS, 255);
	cv::createTrackbar("S min", "HSV Parameters", &minS, 255);
	cv::createTrackbar("V max", "HSV Parameters", &maxV, 255);
	cv::createTrackbar("V min", "HSV Parameters", &minV, 255);
	//cv::resizeWindow("HSV Parameters", 0, 0);
//*********************************************************************
  }

  ~Edge_Detector()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    namespace enc = sensor_msgs::image_encodings;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
	detect_edges(cv_ptr->image);
    	image_pub_.publish(cv_ptr->toImageMsg());

	
  }
  void detect_edges(cv::Mat img)
  {
	IplImage srcImg=img;
	IplImage *dstImg = NULL;
	dstImg = cvCreateImage( cvSize(640,480),8,3);
	cvResize( &srcImg, dstImg, CV_INTER_LINEAR );
	//IplImage ipltemp=img;
	//cvCopy(&ipltemp,image);
// HSV image
	IplImage *hsv = cvCloneImage(dstImg);
	cvCvtColor(dstImg, hsv, CV_RGB2HSV_FULL);
// Binalized image
	IplImage *binalized = cvCreateImage(cvGetSize(dstImg), IPL_DEPTH_8U, 1);
// Binalize
	CvScalar lower = cvScalar(minH, minS, minV);
	CvScalar upper = cvScalar(maxH, maxS, maxV);
	cvInRangeS(dstImg, lower, upper, binalized);
// Show result
	
// *****************************************************************************
// De-noising
	cvMorphologyEx(binalized, binalized, NULL, NULL, CV_MOP_CLOSE);
// Detect contours
	CvSeq *contour = NULL, *maxContour = NULL;
	CvMemStorage *contourStorage = cvCreateMemStorage();
	cvFindContours(binalized, contourStorage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
// Find largest contour
	double max_area = 0.0;
	while (contour) {
		double area = fabs(cvContourArea(contour));
		if (area > max_area) {
			maxContour = contour;
			max_area = area;
		}
		contour = contour->h_next;
	}
// Object detected

	if (maxContour) {
		// Draw a contour
		cvZero(binalized);
		cvDrawContours(binalized, maxContour, cvScalarAll(255), cvScalarAll(255), 0, CV_FILLED);
		// Calculate the moments
		CvMoments moments;
		cvMoments(binalized, &moments, 1);
		my = (int)(moments.m01 / moments.m00);
		mx = (int)(moments.m10 / moments.m00);
		// Measurements
		float m[] = { mx, my };
		CvMat measurement = cvMat(2, 1, CV_32FC1, m);
//******************************************************************************
}
	//int height=480;
	//int width=640;
	cvCircle(dstImg,cv::Point(mx,my),7,CV_RGB(0,255,0),-1);
	//cvCircle(dstImg,cv::Point(320,184),7,CV_RGB(255,255,0),-1);
	//int a1=image.size().width;
	//int a2=image.size().height;
	//cvLine(image,cv::Point(320,0),cv::Point(320,480), cv::Scalar(0,255,0),2,8,0);//up
	//cvLine(image,cv::Point(0,240),cv::Point(640,240), cv::Scalar(0,255,0),2,8,0);//left
	cvShowImage("binalized", binalized);
	cvShowImage("original", dstImg);
	std::cout<<mx<<" "<<my<<"\n";
	cv::waitKey(10);	
	std_msgs::Int32 msgx;
	msgx.data = mx;
	std_msgs::Int32 msgy;
	msgy.data = my;
	mx_pub.publish(msgx);
	my_pub.publish(msgy);
  }	
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "NVision");
  Edge_Detector ic;
  ros::spin();
  return 0;
}
