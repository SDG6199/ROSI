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
#include <cmath>
#include <boost/thread/thread.hpp>  //use thread in ros
#include <image_saver/image_saver.h>

int image_save_flag=1;
int image_save_tag=0;
int captured_car_count=1;
bool is_car=false;
bool is_plastic_person=false;
int flag_illegal=0;
int buzzer_stop_flag=0;
std_msgs::Byte buzzer_tag;

std::vector<std::string> detected_bboxes_class_;

darknet_ros_msgs::BoundingBox car_box;
darknet_ros_msgs::BoundingBox plastic_man_box;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver_node");

  //shell script로 py3_cut_plate.py실행.
  std::string buffer;
  buffer="bash py3_cut_plate.sh &";
  system(buffer.c_str()); 

  ImageSaver ic;
  DetectLine dl;
  boost::thread t1(check_man_illegal);
  //check_man_illegal은 thread로 돌린다.
  ros::spin();
  return 0;
}

void ImageSaver::savetag(const std_msgs::Byte::ConstPtr& msg)
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
}
void ImageSaver::detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
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
      // car_box.xmax=0; car_box.xmin=0;
      // car_box.ymax=0; car_box.ymin=0;  
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
void ImageSaver::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr, cv_ptr_bound_img;
  cv::Mat cut_img, show_cut_img;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    cv_ptr_bound_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //받은 이미지를 opencv에 맞는 format으로 변환한다.
    //const std::string 	BGR8 = "bgr8". 채널당 8bit 메모리 할당. opencv는 bgr 채널 순서를 사용한다!
  }
  catch (cv_bridge::Exception& e) //toCvCopy나 toCvShared를 쓸때는 항상 이런식으로 에러를 잡는다.
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(image_save_tag==1 || image_save_tag==2)
  //차가 발견되었을 때
  {
    if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    {
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
      if(is_car==true && image_save_flag==1)
      {
        //차가 발견되고 image_save_flag가 1일 때 
        char buff[256];
        int offset=5;
        cut_img=cv_ptr->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
        sprintf(buff,"/home/sdg/catkin_ws/src/image_saver/image/car_image/captured car%d.jpg",captured_car_count);        
        if(image_save_tag==1)
        //차가 처음으로 발견될 때 
        {
          // show_cut_img.
          //cv::rectangle(cv_ptr_bound_img->image,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmax,car_box.ymax)), cv::Scalar(0,0,255),2,8,0);
          //cv::putText(cv_ptr_bound_img->image,"Warning",cv::Point(car_box.xmin,car_box.ymin),5,4,cv::Scalar(0,220,255),5,8);
          show_cut_img=cv_ptr_bound_img->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
          cv::imwrite(buff,show_cut_img);
        }
        else if(image_save_tag==2)
        //차가 불법주정차로 의심될 때 
        {
          // show_cut_img.
          //cv::rectangle(cv_ptr_bound_img->image,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),cv::Point(car_box.xmax,car_box.ymax)), cv::Scalar(0,0,255),2,8,0);
          //cv::putText(cv_ptr_bound_img->image,"confirm",cv::Point(car_box.xmin,car_box.ymin),5,4,cv::Scalar(0,220,255),5,8);
          show_cut_img=cv_ptr_bound_img->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
          cv::imwrite(buff,show_cut_img);
        }
        image_save_flag=0;
        //image_save_flag를 끈다.
        captured_car_count++; 
      }
      cv::waitKey(1);

      //image_pub_.publish(cv_ptr->toImageMsg());  //cutting된 이미지를 publish한다.
    }
  }
  ros::Duration(0.1).sleep();
}
void check_man_illegal()
{
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Publisher buzzer_pub = n->advertise<std_msgs::Byte>("/buzzer_tag", 1000);
  ros::Rate loop_rate(10);
  while(ros::ok())
  {   
    if(no_cross_mat.size()!=0 && is_plastic_person==true && buzzer_stop_flag==0)
    //no_cross_mat가 존재하고 사람이 존재하고 buzzer_stop_flag가 0일 때
    {
      printf("no_cross_mat_row: %d, no_cross_mat_col: %d \n",no_cross_mat.size(), no_cross_mat[0].size());
      for(int i=0; i<no_cross_mat.size(); i++)
      {
        for(int j=0; j<no_cross_mat[i].size(); j++)
        {
          float dist_illegal=calc_distance(&no_cross_mat[i][j],plastic_man_box);
          //no_cross_mat의 각 요소와 사람과의 거리 dist_illegal를 계산한다.
          if(dist_illegal<10.0)  //@
          //dist_illegal가 가까우면
          {
            printf("dist_illegal: %f \n",dist_illegal);
            flag_illegal=1;
          }
          if(flag_illegal==1){break;}
        }
        if(flag_illegal==1){break;}
      }
      if(flag_illegal==1)
      {
        flag_illegal=0;
        buzzer_tag.data=1;
        //buzzer를 울린다.
      }
    }
    buzzer_pub.publish(buzzer_tag);
    buzzer_tag.data=0;
    loop_rate.sleep();
  }
}
float calc_distance(Data_position *d1, darknet_ros_msgs::BoundingBox d2)
//d1은 상대좌표, d2는 절대좌표이다.
{
  float box_x_mean=(float)(d2.xmax+d2.xmin)/2;
  float box_y_mean=(float)(d2.ymax+d2.ymin)/2;
  float mat_pix_x=(float)ORG_X+d1->x;
  float mat_pix_y=(float)ORG_Y+d1->y;

  float dist=sqrt(pow((mat_pix_x-box_x_mean),2)+pow((mat_pix_y-box_y_mean),2));
  //객체와 no_cross_mat의 요소간 거리를 계산한다.
  return dist;
}
void ImageSaver::buzzer_stop_cb(const std_msgs::Byte::ConstPtr& msg)
//buzzer기능을 켤껀지 말껀지를 결정한다.
{
  if(msg->data==0)
  {
    buzzer_stop_flag=0;
  }
  else
  {
    buzzer_stop_flag=1;
  }
}
