#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/Byte.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#define PI 3.14159265358932384
cv::Mat canny_img, gray_img, line_img,img_copy;
cv_bridge::CvImagePtr cv_ptr;
std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
darknet_ros_msgs::BoundingBox car_box;
darknet_ros_msgs::BoundingBox pla_man_box;
darknet_ros_msgs::BoundingBox person_box;
int LED_RYG=0;
char buff1[256],buff2[256];

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber traffic_light_sub;
  ros::Subscriber box_sub_;
  ros::Publisher buzzer_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    traffic_light_sub = nh_.subscribe("/traffic_light_tag", 1000, &ImageConverter::traffic_light_cb, this);
    box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::draw_box, this);
    buzzer_pub_ = nh_.advertise<std_msgs::Byte>("/buzzer_tag",1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void traffic_light_cb(const std_msgs::Byte::ConstPtr& msg);
  void draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

  // Draw an example circle on the video stream
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  {
    cv_ptr->image.copyTo(img_copy);
  }
  // Update GUI Window

  // Output modified video stream
  image_pub_.publish(cv_ptr->toImageMsg());
}

void ImageConverter::draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  try
  {
    detected_bboxes_ = msg->bounding_boxes;  //BoundingBox3d[]. 즉, array형태이다.
    int size = detected_bboxes_.size();
    int j=0;

    if (LED_RYG==1)
    {
      cv::rectangle(img_copy,cv::Rect(cv::Point(350,50),\
      cv::Point(650,680)),cv::Scalar(255,0,0),3,8,0);
    }
    else if (LED_RYG==2)
    {
      cv::rectangle(img_copy,cv::Rect(cv::Point(350,50),\
      cv::Point(650,680)),cv::Scalar(0,0,255),3,8,0);
    }


    for(int i=0;i<size;i++)
    {
      if(detected_bboxes_[i].Class=="car")
      {            
        ROS_INFO("Car");
        car_box= detected_bboxes_[i];

        cv::rectangle(img_copy,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),\
        cv::Point(car_box.xmax,car_box.ymax)),cv::Scalar(0,255,0),1,8,0);
        cv::rectangle(img_copy,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),\
        cv::Point(car_box.xmin+230,car_box.ymin+40)),cv::Scalar(0,255,0),cv::FILLED,8,0);
        sprintf(buff1, "car %.3f",car_box.probability);
        cv::putText(img_copy,buff1,cv::Point(car_box.xmin,car_box.ymin+30),5,2,\
        cv::Scalar(0,0,0),2,8,0);//face,scale
      }

      else if(detected_bboxes_[i].Class=="plastic_man")
      {            

        person_box= detected_bboxes_[i];
        ROS_INFO("%d",person_box.xmin);
        cv::rectangle(img_copy,cv::Rect(cv::Point(person_box.xmin,person_box.ymin),\
        cv::Point(person_box.xmax,person_box.ymax)),cv::Scalar(255,0,0),1,8,0);
        cv::rectangle(img_copy,cv::Rect(cv::Point(person_box.xmin,person_box.ymin),\
        cv::Point(person_box.xmin+320,person_box.ymin+40)),cv::Scalar(255,0,0),cv::FILLED,8,0);
        sprintf(buff2, "pedestrian %.3f",person_box.probability);
        cv::putText(img_copy,buff2,cv::Point(person_box.xmin,person_box.ymin+30),5,1.5,\
        cv::Scalar(0,0,0),2,8,0);//face,scale
      }
    }
    cv::imshow(OPENCV_WINDOW, img_copy);
    cv::waitKey(3);


  }

  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

void ImageConverter::traffic_light_cb(const std_msgs::Byte::ConstPtr& msg)
{
    if(msg->data==1)  //red
    {
      LED_RYG=1;
        //횡단불가. @
    }
    else if(msg->data==2)  //yellow
    {
      LED_RYG=2;
   	//just accessory
    }
    else if(msg->data==3)  //green
    {
      LED_RYG=3;
    }
}

