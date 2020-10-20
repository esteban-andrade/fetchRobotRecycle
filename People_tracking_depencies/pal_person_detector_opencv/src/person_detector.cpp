/**
 * @file person_detector.cpp
 * @author Esteban Andrade
 * @brief This file was taken as inpiration from pal robotics and adjusted to send booleans if objects were detected. 
 * @version 0.1
 * @date 2020-10-20
 * @note: original filed obtained from @jordi-pages
 * @
 * 
 */
// PAL headers
#include <pal_detection_msgs/Detections2d.h>

// ROS headers
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>

// OpenCV headers
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Boost headers
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>

// Std C++ headers
#include <vector>

/**
 * @brief The PersonDetector class encapsulating an image subscriber and the OpenCV's CPU HOG person detector
 *
 * @example rosrun person_detector_opencv person_detector image:=/camera/image _rate:=5 _scale:=0.5
 *
 */
class PersonDetector
{
public:
  PersonDetector(ros::NodeHandle &nh,
                 ros::NodeHandle &pnh,
                 double imageScaling = 1.0);
  virtual ~PersonDetector();
  bool status;

protected:
  ros::NodeHandle _nh, _pnh;

  void imageCallback(const sensor_msgs::ImageConstPtr &msg);

  void detectPersons(const cv::Mat &img,
                     std::vector<cv::Rect> &detections);

  void scaleDetections(std::vector<cv::Rect> &detections,
                       double scaleX, double scaleY) const;

  void publishDetections(const std::vector<cv::Rect> &detections) const;

  void publishDebugImage(cv::Mat &img,
                         const std::vector<cv::Rect> &detections) const;

  double _imageScaling;
  mutable cv_bridge::CvImage _cvImgDebug;

  boost::scoped_ptr<cv::HOGDescriptor> _hogCPU;

  image_transport::ImageTransport _imageTransport, _privateImageTransport;
  image_transport::Subscriber _imageSub;
  ros::Time _imgTimeStamp;

  ros::Publisher _detectionPub;
  image_transport::Publisher _imDebugPub;
  ros::Publisher _detectorPub;
};

PersonDetector::PersonDetector(ros::NodeHandle &nh,
                               ros::NodeHandle &pnh,
                               double imageScaling) : _nh(nh),
                                                      _pnh(pnh),
                                                      _imageScaling(imageScaling),
                                                      _imageTransport(nh),
                                                      _privateImageTransport(pnh)
{

  _hogCPU.reset(new cv::HOGDescriptor);
  _hogCPU->setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());

  image_transport::TransportHints transportHint("raw");

  _imageSub = _imageTransport.subscribe("image", 1, &PersonDetector::imageCallback, this, transportHint);
  _imDebugPub = _privateImageTransport.advertise("debug", 1);

  _detectionPub = _pnh.advertise<pal_detection_msgs::Detections2d>("detections", 1);
  _detectorPub = _pnh.advertise<std_msgs::Bool>("status", 1);

  cv::namedWindow("person detections");
}

PersonDetector::~PersonDetector()
{
  cv::destroyWindow("person detections");
}

void PersonDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImageConstPtr cvImgPtr;
  cvImgPtr = cv_bridge::toCvShare(msg);

  _imgTimeStamp = msg->header.stamp;

  cv::Mat img(static_cast<int>(_imageScaling * cvImgPtr->image.rows),
              static_cast<int>(_imageScaling * cvImgPtr->image.cols),
              cvImgPtr->image.type());

  if (_imageScaling == 1.0)
    cvImgPtr->image.copyTo(img);
  else
  {
    cv::resize(cvImgPtr->image, img, img.size());
  }

  std::vector<cv::Rect> detections;

  detectPersons(img, detections);

  if (_imageScaling != 1.0)
  {
    scaleDetections(detections,
                    static_cast<double>(cvImgPtr->image.cols) / static_cast<double>(img.cols),
                    static_cast<double>(cvImgPtr->image.rows) / static_cast<double>(img.rows));
  }

  publishDetections(detections);

  cv::Mat imDebug = cvImgPtr->image.clone();
  publishDebugImage(imDebug, detections);
}

void PersonDetector::scaleDetections(std::vector<cv::Rect> &detections,
                                     double scaleX, double scaleY) const
{
  BOOST_FOREACH (cv::Rect &detection, detections)
  {
    cv::Rect roi(detection);
    detection.x = static_cast<long>(roi.x * scaleX);
    detection.y = static_cast<long>(roi.y * scaleY);
    detection.width = static_cast<long>(roi.width * scaleX);
    detection.height = static_cast<long>(roi.height * scaleY);
  }
}

void PersonDetector::detectPersons(const cv::Mat &img,
                                   std::vector<cv::Rect> &detections)
{
  double start = static_cast<double>(cv::getTickCount());

  _hogCPU->detectMultiScale(img,
                            detections,
                            0,              //hit threshold: decrease in order to increase number of detections but also false alarms
                            cv::Size(8, 8), //win stride
                            cv::Size(0, 0), //padding 24,16
                            1.02,           //scaling
                            1,              //final threshold
                            false);         //use mean-shift to fuse detections

  double stop = static_cast<double>(cv::getTickCount());
  ROS_DEBUG_STREAM("Elapsed time in detectMultiScale: " << 1000.0 * (stop - start) / cv::getTickFrequency() << " ms");
}

void PersonDetector::publishDetections(const std::vector<cv::Rect> &detections) const
{
  pal_detection_msgs::Detections2d msg;
  pal_detection_msgs::Detection2d detection;

  msg.header.frame_id = "";
  msg.header.stamp = _imgTimeStamp;

  BOOST_FOREACH (const cv::Rect &roi, detections)
  {
    detection.x = roi.x;
    detection.y = roi.y;
    detection.width = roi.width;
    detection.height = roi.height;

    msg.detections.push_back(detection);
  }

  std_msgs::Bool detection_status;
  if (msg.detections.empty())
  {
    detection_status.data = true;
  }
  else
  {
    detection_status.data = false;
  }
  // detection_status.data = status;

  _detectionPub.publish(msg);
  _detectorPub.publish(detection_status);
}

void PersonDetector::publishDebugImage(cv::Mat &img,
                                       const std::vector<cv::Rect> &detections) const
{
  //draw detections
  BOOST_FOREACH (const cv::Rect &roi, detections)
  {
    cv::rectangle(img, roi, CV_RGB(0, 255, 0), 2);
  }

  if (img.channels() == 3 && img.depth() == CV_8U)
    _cvImgDebug.encoding = sensor_msgs::image_encodings::RGB8;

  else if (img.channels() == 1 && img.depth() == CV_8U)
    _cvImgDebug.encoding = sensor_msgs::image_encodings::MONO8;
  else
    throw std::runtime_error("Error in Detector2dNode::publishDebug: only 24-bit BGR or 8-bit MONO images are currently supported");

  _cvImgDebug.image = img;
  sensor_msgs::Image imgMsg;
  imgMsg.header.stamp = _imgTimeStamp;
  _cvImgDebug.toImageMsg(imgMsg); //copy image data to ROS message

  _imDebugPub.publish(imgMsg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pal_person_detector_opencv"); // Create and name the Node
  ros::NodeHandle nh, pnh("~");

  ros::CallbackQueue cbQueue;
  nh.setCallbackQueue(&cbQueue);

  double scale = 1.0;
  pnh.param<double>("scale", scale, scale);

  double freq = 1;
  pnh.param<double>("rate", freq, freq);

  ROS_INFO_STREAM("Setting image scale factor to: " << scale);
  ROS_INFO_STREAM("Setting detector max rate to:  " << freq);
  ROS_INFO(" ");

  ROS_INFO_STREAM("Creating person detector ...");

  PersonDetector detector(nh, pnh, scale);

  ROS_INFO_STREAM("Spinning to serve callbacks ...");

  ros::Rate rate(freq);
  while (ros::ok())
  {
    cbQueue.callAvailable();
    rate.sleep();
  }

  return 0;
}
