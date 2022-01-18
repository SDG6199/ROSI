#include <ros/ros.h>
#include "std_msgs/Byte.h"
#include <image_transport/image_transport.h> //package.xml에 추가. 이미지데이터를 받고 보내기위해 사용.
#include <cv_bridge/cv_bridge.h>             //package.xml에 추가
#include <sensor_msgs/image_encodings.h>     //package.xml에 추가. cv_bridge::CvImagePtr에 값을 대입하기 위한 인자로 사용된다. 
#include <opencv2/imgproc/imgproc.hpp>       //OpenCV's image processing를 사용하기위해 필요하다.
#include <opencv2/highgui/highgui.hpp>       //OpenCV's GUI modules를 사용하기위해 필요하다. (cv::namedWindow(OPENCV_WINDOW);)
#include <darknet_ros_msgs/BoundingBoxes.h>  // darknet_ros_msgs::BoundingBoxes를 쓰기위해 필요하다.
#include <darknet_ros_msgs/BoundingBox.h>
#include <stdlib.h>  //system()

static const std::string OPENCV_WINDOW = "Captured Car";
int image_save_tag=0, image_save_flag=1;
int captured_car_count=1;
bool is_car=false;
bool is_plastic_person=false;

std::vector<std::string> detected_bboxes_class_;

darknet_ros_msgs::BoundingBox car_box;
darknet_ros_msgs::BoundingBox plastic_man_box;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;    
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber box_sub_;
  ros::Subscriber save_sub;

  public:
    ImageConverter(): it_(nh_)      //it_에 nh_을 대입한다. 
    {
      save_sub = nh_.subscribe("/image_save_tag",1, &ImageConverter::savetag,this);
      //image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this, image_transport::TransportHints("compressed"));  //멤버함수를 가리키는 함수 포인터는 &ImageConverter::imageCb로 쓴다. 
      box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::detect_box, this); //이 template는 호출한 객체의 주소도 전달해야한다. 생성자 내에 있어서 그런거같다.
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      //cv::namedWindow(OPENCV_WINDOW);
      //cv::resizeWindow(OPENCV_WINDOW, 400,450);
      //cv::moveWindow(OPENCV_WINDOW, 1920, 0);   //내 모니터 해상도: 1920x1080
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void savetag(const std_msgs::Byte::ConstPtr& msg);
    void detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver_node");

  //shell script로 py3_cut_plate.py실행.
  std::string buffer;
  buffer="bash py3_cut_plate.sh &";
  system(buffer.c_str()); 

  ImageConverter ic;
  //DetectLine dl;
  ros::spin();
  return 0;
}

void ImageConverter::savetag(const std_msgs::Byte::ConstPtr& msg)
{
  if(msg->data==1)
  {
    image_save_tag=1;
  }
  else if(msg->data==2)
  {
    image_save_tag=2;
  }
  else
  {
    image_save_tag=0;
    image_save_flag=1;
  }
  ros::Duration(0.1).sleep();
}

void ImageConverter::detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
  detected_bboxes_ = msg->bounding_boxes;  //BoundingBox3d[]. 즉, array형태이다.
  int size =detected_bboxes_.size();
  std::string element_to_check_car="car";
  std::string element_to_check_plastic_man="plastic_man";
  
  for(int i=0;i<size;i++)
  {
    detected_bboxes_class_.insert(detected_bboxes_class_.begin() + i, detected_bboxes_[i].Class);
    if(detected_bboxes_[i].Class=="car")  
    {
        car_box= detected_bboxes_[i];
    }
    if(detected_bboxes_[i].Class=="plastic_man")  
    {
        plastic_man_box= detected_bboxes_[i];
    }
  }

  if (any_of(detected_bboxes_class_.begin(), detected_bboxes_class_.end(), [&](const std::string& elem) { return elem == element_to_check_car; })) 
  {
      //printf("%s is present in the vector\n", element_to_check_car.c_str());
      is_car=true;
  }
  else
  {
      //printf("%s isn't present in the vector\n", element_to_check_car.c_str());
      is_car=false;
      car_box.xmax=0; car_box.xmin=0;
      car_box.ymax=0; car_box.ymin=0;  
  }

  if (any_of(detected_bboxes_class_.begin(), detected_bboxes_class_.end(), [&](const std::string& elem) { return elem == element_to_check_plastic_man; })) 
  {
      //printf("%s is present in the vector\n", element_to_check_plastic_man.c_str());
      is_plastic_person=true;
  }
  else
  {
      //printf("%s isn't present in the vector\n", element_to_check_plastic_man.c_str());
      is_plastic_person=false;
  }

  for(int i=0;i<size;i++)
  {
      detected_bboxes_class_.erase(detected_bboxes_class_.begin() + i);
  }
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr, cv_ptr_bound_img;
  cv::Mat cut_img, show_cut_img;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  //받은 이미지를 opencv를 사용하기위해 변환한다.
    //const std::string 	BGR8 = "bgr8". 채널당 8bit 메모리 할당. opencv는 bgr 채널 순서를 사용한다!
    cv_ptr_bound_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) //toCvCopy나 toCvShared를 쓸때는 항상 이런식으로 에러를 잡아라.
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  
  if(image_save_tag==1 || image_save_tag==2)
  {
    if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      if(is_car==true && image_save_flag==1)
      {
        // captured car저장.
        char buff[256];
        int offset=5;
        cut_img=cv_ptr->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
        sprintf(buff,"/home/sdg/catkin_ws/src/image_saver/image/car_image/captured car%d.jpg",captured_car_count);
        cv::imwrite(buff,cut_img);
        //PyObject로 py3_cut_plate의 cut_plate()의 인자로 이미지 cut_img를 직접보내면 됨.. 하지만 cross compile해야해서 어렵다.(PyObject_CallObject) @
        
        if(image_save_tag==1)
        {
          // show_cut_img.
          cv::rectangle(cv_ptr_bound_img->image,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmax,car_box.ymax)), cv::Scalar(0,0,255),2,8,0);
          cv::putText(cv_ptr_bound_img->image,"Warning",cv::Point(car_box.xmin,car_box.ymin),5,4,cv::Scalar(0,220,255),5,8);
          show_cut_img=cv_ptr_bound_img->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
          //cv::imshow(OPENCV_WINDOW, show_cut_img);   
        }
        else if(image_save_tag==2)
        {
          // show_cut_img.
          cv::rectangle(cv_ptr_bound_img->image,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmax,car_box.ymax)), cv::Scalar(0,0,255),2,8,0);
          cv::putText(cv_ptr_bound_img->image,"confirm",cv::Point(car_box.xmin,car_box.ymin),5,4,cv::Scalar(0,220,255),5,8);
          show_cut_img=cv_ptr_bound_img->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
          //cv::imshow(OPENCV_WINDOW, show_cut_img); 
        }
        image_save_flag=0;
        captured_car_count++; 
      }
      cv::waitKey(1);

      image_pub_.publish(cv_ptr->toImageMsg());  //cutting된 이미지를 publish한다.
    }
  }
  ros::Duration(0.1).sleep();
}