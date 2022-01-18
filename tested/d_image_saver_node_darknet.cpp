#include <ros/ros.h>
#include "std_msgs/Byte.h"
#include <image_transport/image_transport.h> //package.xml에 추가. 이미지데이터를 받고 보내기위해 사용.
#include <cv_bridge/cv_bridge.h>             //package.xml에 추가
#include <sensor_msgs/image_encodings.h>     //package.xml에 추가. cv_bridge::CvImagePtr에 값을 대입하기 위한 인자로 사용된다. 
#include <opencv2/imgproc/imgproc.hpp>       //OpenCV's image processing를 사용하기위해 필요하다.
#include <opencv2/highgui/highgui.hpp>       //OpenCV's GUI modules를 사용하기위해 필요하다. (cv::namedWindow(OPENCV_WINDOW);)
#include <darknet_ros_msgs/BoundingBoxes.h>  // darknet_ros_msgs::BoundingBoxes를 쓰기위해 필요하다.
#include <darknet_ros_msgs/BoundingBox.h>
#include <image_saver/cut_plate.h>
//#include <thread>
 
static const std::string OPENCV_WINDOW = "Captured Car";
int image_save_tag=0, image_save_flag=1;
int captured_car_count=1;
bool is_car=false;
std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
darknet_ros_msgs::BoundingBox car_box;

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
      image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &ImageConverter::imageCb, this);  //멤버함수를 가리키는 함수 포인터는 &ImageConverter::imageCb로 쓴다. 
      box_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::detect_box, this); //이 template는 호출한 객체의 주소도 전달해야한다. 생성자 내에 있어서 그런거같다.
      image_pub_ = it_.advertise("/image_converter/output_video", 1);
      cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConverter()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }
    
    void savetag(const std_msgs::Byte::ConstPtr& msg)
    {
      if(msg->data==1)
      {
        image_save_tag=1;
      }
      else
      {
        image_save_tag=0;
        image_save_flag=1;
      }
    }

    void detect_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
    {
      detected_bboxes_ = msg->bounding_boxes;  //BoundingBox3d[]. 즉, array형태이다.
      int size = detected_bboxes_.size();
      int j=0;
      for(int i=0;i<size;i++)
      {
        if(detected_bboxes_[i].Class=="car")
        {            
          is_car=true;
          car_box= detected_bboxes_[i];
        }
        else
        {
          j++;
        }
      }
      if(j==size)
      {
        is_car=false;
      }

    }
    
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv::Mat cut_img;
      cv_bridge::CvImagePtr cv_ptr_bound_img;

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
      //image_save_tag 메시지를 받았을 때 cut_img저장 및 imshow 한번 실행 ----------
      if(image_save_tag==1)
      {
        if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        {
          cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
          
          if(is_car==true && image_save_flag==1)
          {
            char buff[256];
            int offset=50;
            cut_img=cv_ptr->image(cv::Range(car_box.ymin-offset,car_box.ymax+offset), cv::Range(car_box.xmin-offset,car_box.xmax+offset));
            sprintf(buff,"/home/sdg/catkin_ws/src/image_saver/image/car_image/captured car%d.jpg",captured_car_count);
            cv::imwrite(buff,cut_img);
            
            cv::rectangle(cv_ptr_bound_img->image,cv::Rect(cv::Point(car_box.xmin+5,car_box.ymin+5),cv::Point(car_box.xmax-5,car_box.ymax-5)), cv::Scalar(0,0,255),2,8,0);
            cv::putText(cv_ptr_bound_img->image,"Warning",cv::Point(car_box.xmin,car_box.ymin),5,4,cv::Scalar(0,220,255),5,8);
            cv::imshow(OPENCV_WINDOW, cv_ptr_bound_img->image);  //왜 imshow가 cut_plate보다 느린가? -> system함수가 key입력을해서 opencv_window종료. sh로 명령어실행하면 key입력을 피할 수 있다.
            cv::waitKey(1);
            
            uint8_t ret=cut_plate();  //번호판을 자른다. 실행시간이 5초정도걸리는데 그 사이에 다른차가 인식되면 저장을 못함.@
            if(ret)
            {
                ROS_INFO("%s", "Plate is cut.");
            }
            else
            {
                ROS_ERROR("%s", "Failed to cut plate.");
            }
            //std::thread _t1(cut_plate); 
            //_t1.detach(); 

            captured_car_count++; 
          }
          cv::waitKey(1);

          image_pub_.publish(cv_ptr->toImageMsg());  //cutting된 이미지를 publish한다.
        }
        image_save_flag=0;
      }
      //-------------------------------------------------
    }
  };

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_saver_node");
  ImageConverter ic;

  // /home/sdg/catkin_ws/src/image_saver/image내의 파일 전체 삭제, bboxes_detail.txt삭제 후 
  // car_image, plate_cut_image, plate_detect_image 폴더 생성. https://modoocode.com/306 (파일 / 디렉토리 삭제하기)참고. @

  ros::spin();
  return 0;
}