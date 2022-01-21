#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <darknet_ros_msgs/BoundingBoxes.h>  // darknet_ros_msgs::BoundingBoxes를 쓰기위해 필요하다.
#include <darknet_ros_msgs/BoundingBox.h>
#include "std_msgs/Byte.h"
#include <image_saver/image_saver.h>

#define PI 3.14159265358932384

#define _640_480
//#define _960_1080
//#define _1280_720
//#define GRAY_CAM

#ifdef _640_480
const int FIX_WIDTH = 640;
const int FIX_HEIGHT = 480;
#endif

#ifdef _1280_720
const int FIX_WIDTH = 1280;
const int FIX_HEIGHT = 720;
#endif

#ifdef _960_1080
const int FIX_WIDTH = 960;
const int FIX_HEIGHT = 1080;
#endif

int WIDTH=FIX_WIDTH;
int HEIGHT=FIX_HEIGHT;
int ORG_X=0;
int ORG_Y=0;
int X_HALF=(WIDTH+ORG_X)/2;
int Y_HALF=(HEIGHT+ORG_Y)/2;

float roi_scale_x=1.0/3;  //변경가능@
float roi_scale_y=1.0/10;  //변경가능@

const cv::Scalar RED=cv::Scalar(0,0,255);
const cv::Scalar BLUE=cv::Scalar(255,0,0);
const cv::Scalar GREEN=cv::Scalar(0,255,0);
const cv::Scalar LIGHTCORAL=cv::Scalar(240,128,128);
const cv::Scalar BLACK=cv::Scalar(0,0,0);

cv::Mat img_out;
char buff1[256],buff2[256];
cv::Point pt1, pt2, pt3, pt4;

int high_threshold, low_threshold;
int count_frame,count_pair=0;
int dir_roi=0;

int temp_HEIGHT=FIX_HEIGHT*9*roi_scale_y-10;
float temp_right_x_down=FIX_WIDTH*2*roi_scale_x-10;
float temp_left_x_down=ORG_X+10;
float temp_right_x_up=FIX_WIDTH*2*roi_scale_x-10;
float temp_left_x_up=ORG_X+10;

float temp_left_b=269;
float temp_left_m=-11;
float temp_right_b=470;
float temp_right_m=11;

std::vector<std::vector<Data_position>> no_cross_mat;

void DetectLine::direction_roi_cb(const std_msgs::Byte::ConstPtr& msg)
{
  if (msg->data==1)
  {
    dir_roi=1;  // roi set to left
  }
  else if(msg->data==2)
  {
    dir_roi=2;  // roi set to right
  }
  else
  {
    dir_roi=0;  // default frame
  }
}
void DetectLine::imageCb(const sensor_msgs::ImageConstPtr& msg)
//카메라로부터 image를 받는다.
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
    DetectLine::image_process(cv_ptr->image);
  }
}
void DetectLine::image_process(cv::Mat raw_img)
{
  cv::Mat canny_img, gray_img, line_img, roi_gray_ch3,roi;
  switch(dir_roi) 
  {
    case 1:  //dir_roi==1. left
      WIDTH=FIX_WIDTH*2*roi_scale_x;
      HEIGHT=FIX_HEIGHT*9*roi_scale_y;
      ORG_X=0;
      ORG_Y=FIX_HEIGHT*1*roi_scale_y;
      X_HALF=(WIDTH)/2;
      break;
    case 2:  //dir_roi==2. right
      WIDTH=FIX_WIDTH*2*roi_scale_x;
      HEIGHT=FIX_HEIGHT*9*roi_scale_y;
      ORG_X=FIX_WIDTH*roi_scale_x;
      ORG_Y=FIX_HEIGHT*1*roi_scale_y;    
      X_HALF=(WIDTH)/2;
      printf("ORG_X: %d, WIDTH: %d, X_HALF: %d",ORG_X,WIDTH,X_HALF);
      break;
    default:
      WIDTH=FIX_WIDTH;
      HEIGHT=FIX_HEIGHT;
      ORG_X=0;
      ORG_Y=0;
      X_HALF=WIDTH/2;
  }
  if(dir_roi==0)   // default frame.
  {
    //cv::resize(raw_img, raw_img, cv::Size(960, 1080)); //@
    raw_img.copyTo(img_out);
    //std::cout<<raw_img.cols<<" "<<raw_img.rows<<std::endl;
  }
  else
  {
    roi=raw_img(cv::Rect(ORG_X,ORG_Y,WIDTH,HEIGHT));
    //std::cout<<raw_img.cols<<" "<<raw_img.rows<<std::endl;
    cv::cvtColor(roi,gray_img, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray_img,gray_img,cv::Size(3,3),0);
    //roi에 대해서 전처리를 한다.
    cv::cvtColor(gray_img,roi_gray_ch3, cv::COLOR_GRAY2BGR);  //gray cam때문에 쓰는거.

    cv::Canny(gray_img,canny_img, 100, 200);

    std::vector<cv::Vec4i> linesP;  // HoughLines은 cv::Vec2f에 rho와 theta저장.
    
    cv::HoughLinesP(canny_img,linesP,1,1*(PI/180),180,50,10);
    //canny_img로부터 line을 찾는다.
    #ifdef GRAY_CAM
    line_img=DetectLine::drawHoughLines(roi_gray_ch3,linesP);
    line_img.copyTo(img_out);
    #endif
    #ifndef GRAY_CAM
    std::vector<cv::Vec4i> left_lines,right_lines;

    split_left_right(linesP,left_lines,right_lines);
    //linesP로부터 도로의 좌측경계선 벡터와 우측경계선 벡터를 얻는다.
    find_line(raw_img,roi,left_lines,right_lines);
    //도로의 경계가 인식된 경우, raw_img의 roi에 표현된다.
    raw_img.copyTo(img_out);  
    //raw_img를 opencv에 출력할 이미지 img_out에 복사한다.
    #endif
  }
}
cv::Mat DetectLine::drawHoughLines(cv::Mat& img,std::vector<cv::Vec4i> lines)
{
  for (int i = 0; i < lines.size(); i++)
  {
    cv::Vec4i L= lines[i];
    cv::line(img, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]), RED, 1, cv::LINE_AA);
    //인식된 선분을 img에 표시한다.
  }
  return img;
}
void split_left_right(std::vector<cv::Vec4i> lines, std::vector<cv::Vec4i> &left_lines,std::vector<cv::Vec4i> &right_lines )
{
  std::vector<float> gradients;
  std::vector<cv::Vec4i> new_lines;
  for (int i = 0; i < lines.size(); i++)
  {
    int x1=lines[i][0];
    int y1=lines[i][1];
    int x2=lines[i][2];
    int y2=lines[i][3];
    float gradi;
    if(x1-x2==0)
    //선분의 양끝의 x좌표가 동일하면 
    {
      gradi=999.0;
      //gradi의 상한값을 정한다.
    }
    else
    {
      gradi=(float)(y2-y1)/(x2-x1);
      //주어진 좌표에 대한 gradi를 계산한다.
    }
    if(abs(gradi)>0.45)    
    //특정각도 이상인 line인 경우
    {
      printf("gradi: %f, x1: %d, x2: %d, X_HALF: %d \n",gradi,x1,x2,X_HALF); 
      gradients.push_back(gradi);
      //해당 gradi를 gradients 벡터에 push_back한다.
      new_lines.push_back(lines[i]);
      //해당 lines의 요소를 벡터에 push_back한다.
    }
  }
  for(int i=0;i<new_lines.size(); i++)
  {
    cv::Vec4i line=new_lines[i];
    float gradi=gradients[i];
    int x1=line[0];
    int y1=line[1];
    int x2=line[2];
    int y2=line[3];

    if(gradi>0 && x1>X_HALF+WIDTH*(1.0/5) && x2>X_HALF+WIDTH*(1.0/5))  
    //gradi>0이면 우측에 위치한 도로 경계이다. 중앙선은 선 인식에서 제외한다.
    //gradi>0이고, 중앙선이 아닌 경우 line을 벡터에 push_back한다.
    {
      right_lines.push_back(line);
    }
    else if(gradi<0 && x1<X_HALF-WIDTH*(1.0/5) && x2<X_HALF-WIDTH*(1.0/5))
    //gradi<0이면 좌측에 위치한 도로 경계이다. 중앙선은 선 인식에서 제외한다.
    //gradi<0이고, 중앙선이 아닌 경우 line을 벡터에 push_back한다.
    {
      left_lines.push_back(line);
    }
  }
}
void find_line(cv::Mat& r_img, cv::Mat& img, std::vector<cv::Vec4i> &left_lines,std::vector<cv::Vec4i> &right_lines)
{
  float left_b, right_b, left_m, right_m;
  float left_x_down, left_x_up, right_x_down, right_x_up=0;
  
  cv::rectangle(img, cv::Rect(3,3,WIDTH,HEIGHT),BLACK,1,8,0);  //draw roi boundary.
  cv::circle(img,cv::Point(X_HALF+WIDTH*(1.0/5),HEIGHT),10,BLACK,1,8,0);  //@ 중간선 무시 지점.
  cv::circle(img,cv::Point(X_HALF-WIDTH*(1.0/5),HEIGHT),10,BLACK,1,8,0);  //@ 중간선 무시 지점.

  //------------------------------------
  bool draw_left=find_line_params(left_lines,&left_m,&left_b);
  //line을 y=mx+b의 형태로 고려하고, 매개변수 m과 b를 찾는다. 
  //성공여부에 따라 return값을 boolean으로 받는다.
  if(draw_left) 
  {
    left_x_down=(-left_b)/left_m;
    left_x_up=(HEIGHT-left_b)/left_m;
    cv::line(img, cv::Point(left_x_down,0), cv::Point(left_x_up,HEIGHT), BLUE, 3);
    //roi내에 인식된 선을 표시한다.
    temp_left_x_down=left_x_down;
    temp_left_x_up=left_x_up;
    temp_HEIGHT=HEIGHT;
    temp_left_b=left_b;
    temp_left_m=left_m;
    //일시적으로 직선을 표시하기 위한 정보를 저장한다.
  }
  else
  {
    printf("No Left Line. \n");
  }
  bool draw_right=find_line_params(right_lines,&right_m,&right_b);
  //line을 y=mx+b의 형태로 고려하고, 매개변수 m과 b를 찾는다. 
  //성공여부에 따라 return값을 boolean으로 받는다.
  if(draw_right) 
  //line인식에 성공한 경우 
  {
    right_x_down=(-right_b)/right_m;
    right_x_up=(HEIGHT-right_b)/right_m;
    cv::line(img, cv::Point(right_x_down,0), cv::Point(right_x_up,HEIGHT), RED, 3);
    //roi내에 인식된 선을 표시한다. 
    temp_right_x_down=right_x_down;
    temp_right_x_up=right_x_up;
    temp_HEIGHT=HEIGHT;
    temp_right_b=right_b;
    temp_right_m=right_m;
    //일시적으로 직선을 표시하기 위한 정보를 저장한다.
    //즉, temp정보를 갱신한다.
  }
  else
  //line인식에 실패한 경우
  {
    printf("No Right Line. \n");
  }
  
  cv::Point points[1][4]; 
  points[0][0] = cv::Point(temp_left_x_down,0);
  points[0][1] = cv::Point(temp_left_x_up,temp_HEIGHT);
  points[0][2] = cv::Point(temp_right_x_up,temp_HEIGHT);
  points[0][3] =cv::Point(temp_right_x_down,0);

  const cv::Point* pt[1] = { points[0] };
  int npt[] = { 4 };
  cv::Mat green_mask = cv::Mat::zeros(img.size(),CV_8UC3); //CV_8U:0. type: 자료형+(channel(3)-1)*8. cv::Mat 객체를 만들면 default 자료형이 CV_8U이다. 
  //img size만큼 mask를 정의한다.
  cv::fillPoly(green_mask,pt,npt,1,GREEN);
  //도로의 양쪽 경계가 인식된 경우, 그 사이를 영역으로 하는 green_mask를 정의한다.
  cv::addWeighted(img, 1, green_mask, 0.2, 0.0, img);
  //원본 img에 green_mask를 특정 가중치로 더한다.

  //no_cross_mat set.
  float gap=5;  //수정가능 @
  int x_col=(int)(temp_right_x_up-temp_left_x_up)/gap;
  int y_low=(int)temp_HEIGHT/gap;
  int i,j;
      
  no_cross_mat.assign(y_low, std::vector<Data_position>(x_col));
  //no_cross_mat의 size를 할당한다.
  Data_position data_position;

  for(i=0; i<y_low; i++)
  {
    data_position.x=((data_position.y-temp_left_b)/temp_left_m); 
    //data_position.y에 대한 data_position.x를 계산한다. 
    x_col =(((data_position.y-temp_right_b)/temp_right_m)-((data_position.y-temp_left_b)/temp_left_m))/gap;
    //no_cross_mat의 x축 size를 다시 계산한다.
    for(j=0; j<x_col; j++)
    {
      no_cross_mat[i][j].x=data_position.x;
      no_cross_mat[i][j].y=data_position.y;
      data_position.x+=gap;
      //no_cross_mat의 특정 행의 각 열에 data_position을 대입한다.
    }
    data_position.y+=gap;
    //no_cross_mat의 각 행에 data_position을 대입한다.
  }
  
  for(int i=0; i<no_cross_mat.size(); i++)
  {
    for(int j=0; j<no_cross_mat[i].size(); j++)
    {
      cv::circle(img,cv::Point(no_cross_mat[i][j].x,no_cross_mat[i][j].y),3,LIGHTCORAL,1,8,0);
      //no_cross_mat의 각 요소를 표시한다.
    }
  }

  r_img(cv::Rect(ORG_X,ORG_Y,WIDTH,HEIGHT))=img;
  //r_img의 특정 roi에 line과 no_cross_mat가 표시된 img를 덮어쓴다.
}
bool find_line_params(std::vector<cv::Vec4i> &left_lines, float* left_m, float* left_b )  //left로 적은거지 right도 된다.
//y=mx+b의 형태로 매개변수를 찾는다.
{
  float left_avg_x=0, left_avg_y=0, left_avg_gradi=0;
  if(left_lines.size()==0)
    return false;
  for(int i=0; i<left_lines.size(); i++)
  {
    left_avg_x+=left_lines[i][0];
    left_avg_x+=left_lines[i][2];
    left_avg_y+=left_lines[i][1];
    left_avg_y+=left_lines[i][3];
    left_avg_gradi+=(float)(left_lines[i][3]-left_lines[i][1])/(left_lines[i][2]-left_lines[i][0]);    
  }
  left_avg_x=left_avg_x/(left_lines.size()*2);
  //선분의 '양끝'의 x값 평균을 계산한다. 
  left_avg_y=left_avg_y/(left_lines.size()*2);
  //선분의 '양끝'의 y값 평균을 계산한다. 
  left_avg_gradi=left_avg_gradi/left_lines.size();
  //gradient의 평균값을 계산한다.
  *left_m=left_avg_gradi;
  //m값을 저장한다.
  *left_b=left_avg_y-left_avg_gradi*left_avg_x;
  //b값을 저장한다.
  return true;
}
void DetectLine::draw_box(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  std::vector<darknet_ros_msgs::BoundingBox> detected_bboxes_;
  darknet_ros_msgs::BoundingBox car_box;
  darknet_ros_msgs::BoundingBox plastic_man_box;
  if(img_out.rows!=0)  
  {
    try
    {
      if(dir_roi==1 || dir_roi ==2)
      //주행로봇이 left또는 right로 이동중인 경우
      { 
      cv::line(img_out, cv::Point(ORG_X+temp_left_x_down,ORG_Y), cv::Point(ORG_X+temp_left_x_up,ORG_Y+temp_HEIGHT), BLUE, 3);  
      cv::line(img_out, cv::Point(ORG_X+temp_right_x_down,ORG_Y), cv::Point(ORG_X+temp_right_x_up,ORG_Y+temp_HEIGHT), RED, 3);
      //이전에 인식된 line을 표시한다.
      }
      else
      //주행로봇이 수직이동중인 경우
      {
        no_cross_mat.clear();
      }
      detected_bboxes_ = msg->bounding_boxes; 
      int size = detected_bboxes_.size();
      for(int i=0;i<size;i++)
      {
        if(detected_bboxes_[i].Class=="car")
        //객체 car이 인식된 경우
        {            
          car_box= detected_bboxes_[i];

          cv::rectangle(img_out,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),\
          cv::Point(car_box.xmax,car_box.ymax)),cv::Scalar(0,255,0),1,8,0);
          cv::rectangle(img_out,cv::Rect(cv::Point(car_box.xmin,car_box.ymin),\
          cv::Point(car_box.xmin+(car_box.xmax-car_box.xmin)*0.25,car_box.ymin-40)),cv::Scalar(0,255,0),cv::FILLED,8,0);   
          sprintf(buff1, "car %.3f",car_box.probability);
          cv::putText(img_out,buff1,cv::Point(car_box.xmin,car_box.ymin-15),0,1.3, cv::Scalar(0,0,0),2,8,0);
          //해당 객체의 bbox를 표시한다.
        }

        else if(detected_bboxes_[i].Class=="plastic_man")
        //객체 plastic_man이 인식된 경우
        {            
          plastic_man_box= detected_bboxes_[i];

          cv::rectangle(img_out,cv::Rect(cv::Point(plastic_man_box.xmin,plastic_man_box.ymin),\
          cv::Point(plastic_man_box.xmax,plastic_man_box.ymax)),cv::Scalar(255,0,0),1,8,0);
          cv::rectangle(img_out,cv::Rect(cv::Point(plastic_man_box.xmin,plastic_man_box.ymin),\
          cv::Point(plastic_man_box.xmin+(plastic_man_box.xmax-plastic_man_box.xmin)*1.0,plastic_man_box.ymin-40)),cv::Scalar(255,0,0),cv::FILLED,8,0);
          sprintf(buff2, "pedestrian %.3f",plastic_man_box.probability);
          cv::putText(img_out,buff2,cv::Point(plastic_man_box.xmin,plastic_man_box.ymin-15),0,1.0,cv::Scalar(0,0,0),2,8,0);
          //해당 객체의 bbox를 표시한다.
        }
      }
      //cv::resize(img_out, img_out, cv::Size(960, 1080));
      cv::imshow(OPENCV_WINDOW, img_out);
      //cv::resizeWindow(OPENCV_WINDOW, 960, 1080); //@
      cv::waitKey(1);
    }
    catch(const std::exception& e)   
    {
      std::cerr << e.what() << '\n';
    }
  }
}






