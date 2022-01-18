#include <ros/ros.h>
#include <image_transport/image_transport.h> //package.xml에 추가. 이미지데이터를 받고 보내기위해 사용.
#include <cv_bridge/cv_bridge.h>             //package.xml에 추가
#include <sensor_msgs/image_encodings.h>     //package.xml에 추가. cv_bridge::CvImagePtr에 값을 대입하기 위한 인자로 사용된다. 
#include <opencv2/imgproc/imgproc.hpp>       //OpenCV's image processing를 사용하기위해 필요하다.
#include <opencv2/highgui/highgui.hpp>       //OpenCV's GUI modules를 사용하기위해 필요하다. (cv::namedWindow(OPENCV_WINDOW);)

int count=1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img;

  public:
    ImageConverter(): it_(nh_)      //it_에 nh_을 대입한다. 
    {
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      img=cv_ptr->image;
      char buff[256];
      sprintf(buff,"/home/sdg/catkin_ws/src/make_training_set/dataset/num%d.jpg",count);
      cv::imwrite(buff,img);
      count++;       
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "make_training_set");
  ImageConverter ic;
  
  ros::spin();
  return 0;
}