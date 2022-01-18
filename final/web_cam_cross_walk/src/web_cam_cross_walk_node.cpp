
#include <ros/ros.h>
#include "std_msgs/Byte.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <boost/thread/thread.hpp> 

const cv::Scalar RED=cv::Scalar(0,0,255);
const cv::Scalar GREEN=cv::Scalar(0,255,0);

cv::Mat canny_img, gray_img, line_img;
cv_bridge::CvImagePtr cv_ptr;
std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
darknet_ros_msgs::BoundingBox car_box;
darknet_ros_msgs::BoundingBox plastic_man_box;
darknet_ros_msgs::BoundingBox person_box;

int LED_RYG=0;
cv::Mat img_copy, img_out;
char buff1[256],buff2[256];
std_msgs::Byte buzzer_stop_tag;
bool is_car,is_plastic_person=false;
int flag_legal=0;

int ORG_X=350;
int ORG_Y=50;
int WIDTH=300;
int HEIGHT=630;

struct Data_position
{
  int x=0;
  int y=0;
};

std::vector<std::vector<Data_position>> cross_mat;
std::vector<std::string> detected_bboxes_class_;

static const std::string OPENCV_WINDOW = "Webcam window";

class WebcamCrossWalk
{
  ros::NodeHandle n_;
  ros::NodeHandlePtr nh_ = boost::make_shared<ros::NodeHandle>();
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber traffic_light_sub;
  ros::Subscriber box_sub_;

public:
  WebcamCrossWalk()
  : it_(n_)
  {
    // Subscrive to input video feed and publish output video feed    
    traffic_light_sub = nh_->subscribe("/traffic_light_tag", 1000, &WebcamCrossWalk::traffic_light_cb, this);
    box_sub_ = nh_->subscribe("/usb_cam/darknet_ros/bounding_boxes", 1, &WebcamCrossWalk::detect_box, this); //이 template는 호출한 객체의 주소도 전달해야한다. 생성자 내에 있어서 그런거같다.
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &WebcamCrossWalk::imageCb, this);  
    box_sub_ = nh_->subscribe("/usb_cam/darknet_ros/bounding_boxes", 1, &WebcamCrossWalk::draw_box, this);
    
    cv::namedWindow(OPENCV_WINDOW);
  }
  ~WebcamCrossWalk()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  void traffic_light_cb(const std_msgs::Byte::ConstPtr& msg);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);   
  void detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

};
void check_man_cross();
float calc_distance(Data_position *d1, darknet_ros_msgs::BoundingBox d2);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Webcam_cross_walk_node");
  WebcamCrossWalk wc;
  boost::thread t1(check_man_cross);

  ros::spin();
  return 0;
}

void WebcamCrossWalk::traffic_light_cb(const std_msgs::Byte::ConstPtr& msg)
{
    if(msg->data==1)  //red
    {
      LED_RYG=1;
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
void WebcamCrossWalk::detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
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
void WebcamCrossWalk::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  {
    cv_ptr->image.copyTo(img_copy);
  }
}

void WebcamCrossWalk::draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
  darknet_ros_msgs::BoundingBox car_box;
  darknet_ros_msgs::BoundingBox plastic_man_box;

  if(img_copy.rows!=0)  
  {
    try
    {
      if (LED_RYG==1)   //red
      {
        cv::rectangle(img_copy,cv::Rect(ORG_X,ORG_Y,WIDTH,HEIGHT),RED,3,8,0);
      }
      else if (LED_RYG==3)  //green
      {
        cv::rectangle(img_copy,cv::Rect(cv::Point(350,50),cv::Point(650,680)),GREEN,3,8,0);
        // cross_mat set.
        float gap=5; 
        int x_col=(int)WIDTH/gap;
        int y_low=(int)HEIGHT/gap;
        int i,j;
            
        cross_mat.assign(y_low, std::vector<Data_position>(x_col));
        
        Data_position data_position;
        data_position.y=ORG_Y;
        
        for(i=0; i<y_low; i++)
        {
          data_position.x=ORG_X; 
          x_col =WIDTH/gap;
          for(j=0; j<x_col; j++)
          {
            cross_mat[i][j].x=data_position.x;
            cross_mat[i][j].y=data_position.y;
            data_position.x+=gap;
          }
          data_position.y+=gap;
        }
        //삭제가능@
        for(int i=0; i<cross_mat.size(); i++)
        {
          for(int j=0; j<cross_mat[i].size(); j++)
          {
            cv::circle(img_copy,cv::Point(cross_mat[i][j].x,cross_mat[i][j].y),3,GREEN,1,8,0);
          }
        }
      }
      else
      {
        cross_mat.clear();
      }
      detected_bboxes_ = msg->bounding_boxes; 
      int size = detected_bboxes_.size();
      for(int i=0;i<size;i++)
      {
        if(detected_bboxes_[i].Class=="car")
        {            
          car_box= detected_bboxes_[i];

          cv::rectangle(img_copy,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmax,car_box.ymax)),cv::Scalar(0,255,0),1,8,0);
          cv::rectangle(img_copy,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmin+(car_box.xmax-car_box.xmin)*0.25,car_box.ymin-40)),cv::Scalar(0,255,0),cv::FILLED,8,0);   
          sprintf(buff1, "car %.3f",car_box.probability);
          cv::putText(img_copy,buff1,cv::Point(car_box.xmin,car_box.ymin-15),0,1.3, cv::Scalar(0,0,0),2,8,0);//face,scale
        }

        else if(detected_bboxes_[i].Class=="plastic_man")
        {            
          plastic_man_box= detected_bboxes_[i];

          cv::rectangle(img_copy,cv::Rect(cv::Point(plastic_man_box.xmin,plastic_man_box.ymin),\
          cv::Point(plastic_man_box.xmax,plastic_man_box.ymax)),cv::Scalar(255,0,0),1,8,0);
          cv::rectangle(img_copy,cv::Rect(cv::Point(plastic_man_box.xmin,plastic_man_box.ymin),\
          cv::Point(plastic_man_box.xmin+(plastic_man_box.xmax-plastic_man_box.xmin)*1.0,plastic_man_box.ymin-40)),cv::Scalar(255,0,0),cv::FILLED,8,0);
          sprintf(buff2, "pedestrian %.3f",plastic_man_box.probability);
          cv::putText(img_copy,buff2,cv::Point(plastic_man_box.xmin,plastic_man_box.ymin-15),0,1.0,cv::Scalar(0,0,0),2,8,0);//face,scale
        }
      }
      img_out=img_copy;
      cv::imshow("webcam_window", img_out);
      cv::waitKey(1);
    }
    catch(const std::exception& e)   
    {
      std::cerr << e.what() << '\n';
    }
  }
}
void check_man_cross()
{
  ros::NodeHandlePtr nh_ = boost::make_shared<ros::NodeHandle>();
  ros::Publisher buzzer_stop_pub = nh_->advertise<std_msgs::Byte>("/buzzer_stop", 1000);  
  ros::Rate loop_rate(10);
  while(ros::ok())
  {   
    if(cross_mat.size()!=0 && is_plastic_person==true)
    {
      printf("cross_mat_row: %d, cross_mat_col: %d \n",cross_mat.size(), cross_mat[0].size());
      for(int i=0; i<cross_mat.size(); i++)
      {
        for(int j=0; j<cross_mat[i].size(); j++)
        {
          float dist_legal=calc_distance(&cross_mat[i][j],plastic_man_box);
          if(dist_legal<30.0)  //변경가능 @
          {
            printf("dist_legal: %f \n",dist_legal);
            flag_legal=1;
          }
          if(flag_legal==1){break;}
        }
        if(flag_legal==1){break;}
      }
      if(flag_legal==1)
      {
        flag_legal=0;
        buzzer_stop_tag.data=1;
      }
    }
    buzzer_stop_pub.publish(buzzer_stop_tag);
    buzzer_stop_tag.data=0;
    loop_rate.sleep();
  }
}
float calc_distance(Data_position *d1, darknet_ros_msgs::BoundingBox d2)
{
  float box_x_mean=(float)(d2.xmax+d2.xmin)/2;
  float box_y_mean=(float)(d2.ymax+d2.ymin)/2;
  float mat_pix_x=(float)ORG_X+d1->x;
  float mat_pix_y=(float)ORG_Y+d1->y;

  float dist=sqrt(pow((mat_pix_x-box_x_mean),2)+pow((mat_pix_y-box_y_mean),2));
  //printf("%f %f %f %f %f \n",box_x_mean, box_y_mean, mat_pix_x, mat_pix_y, dist);
  return dist;
}