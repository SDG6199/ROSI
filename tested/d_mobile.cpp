//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "std_msgs/String.h"
//#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <darknet_ros_msgs/BoundingBoxes.h> 
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <python2.7/Python.h>   
//#include <thread>
#include <boost/thread/thread.hpp>  //use thread in ros

#define pi 3.141592653589

int pole_tag=0;
int motor_velocity=200;
double diameter_wheel= 0.0326;    //m  @
double to_rad_coef=pi/2048;
bool is_car=false;
bool is_plastic_person=false;
bool is_pix_in_tor=false;

std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> detected_bboxes_; //vector로 변환 
std::vector<std::string> detected_bboxes_class_;

gb_visual_detection_3d_msgs::BoundingBox3d car_box;
gb_visual_detection_3d_msgs::BoundingBox3d plastic_man_box;
double pix_x,pix_y,pix_z; 
int state_store_flag=1;
int stay_for_store_flag=0;
std::vector<double> pix_car_position(3,0);
std::vector<double> toler_pix_car_position(3,0.5);
std::vector<double> prev_pix_car_position(3,0);
std::vector<double> tor_plus_vector(3,0);
std::vector<double> tor_minus_vector(3,0);

double pos,prev_pos,prev_pos_rad,pos_rad,distance,prev_distance;
double toler_distance=0.1;

std_msgs::Byte savetag;
std_msgs::Byte buzzer_tag;

void pole1_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1){pole_tag=1;}
    else{pole_tag=0;}
}
void pole2_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1){pole_tag=2;}
    else{pole_tag=0;}
}
void pole3_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1){pole_tag=3;}
    else{pole_tag=0;}
}
void pole4_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1){pole_tag=4;}
    else{pole_tag=0;}
}
/*
template<typename T>
std::string getType(T) {
    std::string type = "unknown";
    if (std::is_same<T, int>::value) type = "int";
    if (std::is_same<T, double>::value) type = "double";
    if (std::is_same<T, float>::value) type = "float";
    if (std::is_same<T, bool>::value) type = "bool";
    if (std::is_same<T, std::string>::value) type = "string";

    return type;
}

std::cout<<getType(detected_bboxes_[i].Class)<<std::endl;
*/
void detect_box(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
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
        car_box.zmax=0; car_box.zmin=0; 
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
bool pix_in_tor(std::vector<double> pix, std::vector<double> prev, std::vector<double> tor)
{
    std::vector<double> tor_plus(3,0);
    std::vector<double> tor_minus(3,0);
    
    for(const auto i:prev)
    {
        tor_plus[i]=prev[i]+tor[i];
        tor_minus[i]=prev[i]-tor[i];

        if(tor_minus[i]<pix[i] && pix[i]<tor_plus[i])
        {
            is_pix_in_tor=true;
        }
    
    }
    return is_pix_in_tor;
}

void thread_save_mode()  
{
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Publisher save_pub = n->advertise<std_msgs::Byte>("/image_save_tag", 1000);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        save_pub.publish(savetag);
        loop_rate.sleep();
    }
}
void thread_detect_plastic_person()
{
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    //횡단불가영역 침범 여부 토픽 수신하는 subscriber   @ 사실 detect_line에서 하면 된다.
    ros::Publisher buzzer_pub = n->advertise<std_msgs::Byte>("/buzzer_tag", 1000);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {   
        if(is_plastic_person==true)  
        {
            /*
            if(횡단불가영역 침범 여부 토픽==1)  @ 사실 detect_line에서 하면 된다.
            {
                buzzer_tag.data=1;
            }
            */
            buzzer_tag.data=1;
        }
        else
        {
            buzzer_tag.data=0;
        }
        buzzer_pub.publish(buzzer_tag);
        loop_rate.sleep();
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile");
    //ros::NodeHandle n; 
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();

    ros::Subscriber sub1 = n->subscribe("pole1_sensor", 1000, pole1_callback);
    ros::Subscriber sub2 = n->subscribe("pole2_sensor", 1000, pole2_callback);
    ros::Subscriber sub3 = n->subscribe("pole3_sensor", 1000, pole3_callback);
    ros::Subscriber sub4 = n->subscribe("pole4_sensor", 1000, pole4_callback);
    ros::Subscriber car_box_sub = n->subscribe("/darknet_ros_3d/bounding_boxes", 1000, detect_box);     

    ros::Rate rate(10);
    
    ros::Duration turn_time= ros::Duration(5);
    ros::Time prev_time=ros::Time::now();
    ros::Time current_time=ros::Time::now();
    ros::Time isnot_car_time=ros::Time::now();
    ros::Time stored_time=ros::Time::now();

    PyObject *pName,*pValue,*pModule,*SET_VEL, *GET_STATUS, *CLEAR;
    PyObject *pArgs;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"/home/sdg/catkin_ws/src/rosi_controller/script\")");
    pArgs = PyTuple_New(2);

    pName = PyUnicode_FromString("velocity_mode_read_status");  //PyObject 생성.
    pModule = PyImport_Import(pName); //생성한 PyObject pName을 import한다.

    SET_VEL = PyObject_GetAttrString(pModule, "set_vel"); //실행할 함수를 PyObject에 전달.
    GET_STATUS = PyObject_GetAttrString(pModule, "get_status");
    CLEAR = PyObject_GetAttrString(pModule, "clear");
    //pValue = PyObject_CallObject(pFunc, NULL); //매개변수(pArgs)를 전달하며 pFunc를 호출한다. 현재 매개변수가 NULL인 경우이다. return값을 받으려면 PyLong_AsLong을 써야한다.
    
    boost::thread t1(thread_save_mode);
    boost::thread t2(thread_detect_plastic_person);
    //std::thread t1(thread_save_mode);
    
    /*
    pValue = PyObject_CallObject(GET_STATUS, NULL);
    pos = PyLong_AsLong(pValue);    //rad아니다. 4byte row data(-2,147,483,648 to 2,147,483,647)이다.
    prev_pos_rad = pos*to_rad_coef; 
    double prev_degree=prev_pos_rad*(180/pi);
    */ 
    while(ros::ok())
    {
        /*
        pValue = PyObject_CallObject(GET_STATUS, NULL);
        pos = PyLong_AsLong(pValue);    //rad아니다. 4byte row data(-2,147,483,648 to 2,147,483,647)이다.
        pos_rad = pos*to_rad_coef; 
        int degree=pos_rad*(180/pi);
        degree=degree-prev_degree;

        printf("%d \n", degree);
        if(0<=degree%360&&degree%360<=5)
        {
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs);
            ros::Duration(3).sleep();
        }
        */
        if(pole_tag==1)
        {/*
            if(도착시)   //홀짝으로구분 ㄱ
            {
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //5초 대기
            }
            else    //출발시
            {
                ros::Duration(3).sleep();  //3초 대기
                PyTuple_SetItem(pArgs, 0, PyLong_FromLong(motor_velocity)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(-motor_velocity));
                PyObject_CallObject(SET_VEL, pArgs); 
            }
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;*/
        
        }
        else if(pole_tag==2)  //도착시 정지.
        {
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //5초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;
        }
        else if(pole_tag==3)  //도착시 정지.
        {
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //5초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;
        }
        else if(pole_tag==4)  //도착시 정지.
        {
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //5초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;
        }
        else
        {
            //default 
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(motor_velocity)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(-motor_velocity));
            PyObject_CallObject(SET_VEL, pArgs);

            if(is_car==true)
            {
                current_time=ros::Time::now(); 

                while(current_time-isnot_car_time<ros::Duration(2))
                {
                    current_time=ros::Time::now();

                    PyTuple_SetItem(pArgs, 0, PyLong_FromLong(motor_velocity/2)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(-motor_velocity/2));
                    PyObject_CallObject(SET_VEL, pArgs);  
                    if(car_box.xmax!=0)  //@
                    {
                        //store previous pixel coordinate
                        pix_car_position[0]=(car_box.xmax-car_box.xmin)/2.0;   
                        pix_car_position[1]=(car_box.ymax-car_box.ymin)/2.0;
                        pix_car_position[2]=(car_box.zmax-car_box.zmin)/2.0;
                    }
                    rate.sleep();
                    ros::spinOnce();

                }

                if(state_store_flag==1)
                {
                    stored_time=ros::Time::now();
                
                    prev_pix_car_position=pix_car_position;
                    //printf("x: %f y: %f z: %f \n",pix_car_position[0], pix_car_position[1] ,pix_car_position[2]);    

                    //store previous distance
                    pValue = PyObject_CallObject(GET_STATUS, NULL);
                    pos = PyLong_AsLong(pValue);   

                    pos_rad = pos*to_rad_coef;      
                    distance=(pos_rad-prev_pos_rad)*diameter_wheel*pi;    //m
                    printf("distance %f \n",distance);

                    prev_distance=distance;

                    savetag.data=1;
                    state_store_flag=0;
                }
                else   //비교주행
                {
                    current_time=ros::Time::now();
                    
                    //store present position
                    pValue = PyObject_CallObject(GET_STATUS, NULL);
                    pos = PyLong_AsLong(pValue);    
                    pos_rad = pos*to_rad_coef;      
                    distance=(pos_rad-prev_pos_rad)*diameter_wheel*pi;    //m
                    
                    is_pix_in_tor=pix_in_tor(pix_car_position,prev_pix_car_position,toler_pix_car_position);
                    
                    if(prev_distance-toler_distance<distance<prev_distance+toler_distance && \
                    is_pix_in_tor && (current_time-stored_time)>ros::Duration(5))
                    {
                        stored_time=ros::Time::now();
                        prev_pix_car_position=pix_car_position;
                        prev_distance=distance;

                        savetag.data=2;  
                        stay_for_store_flag=1; 
                        is_pix_in_tor=false; 
                    }
                } 
                
            }
            else
            {   
                prev_time=ros::Time::now();

                if(prev_time-current_time>ros::Duration(1))
                {
                    isnot_car_time=ros::Time::now();
                    savetag.data=0;
                    if(stay_for_store_flag==1)
                    {
                        state_store_flag=1;
                        stay_for_store_flag=0;
                    }
                }
            }
            /*
            중앙선으로부터 좌우로 횡단불가영역 픽셀로 표시.
            해당 영역을 마킹해서 yolo랑 따로 streaming.
            // store pixel coordinate(human)
            pix_human_x=(human_box.xmax-human_box.xmin)/2.0;
            pix_human_y=(human_box.ymax-human_box.ymin)/2.0;
            pix_human_z=(human_box.zmax-human_box.zmin)/2.0;
            pix_human_position(vector)에 pix_human_x,pix_human_y,pix_human_z 대입.

            if(pix_human_position이 횡단불가영역에 포함된다)  //@
            {
                부저울림토픽 publish.
            }
            */
        }

        rate.sleep();
        ros::spinOnce();
    }
    
    PyObject_CallObject(CLEAR, NULL);
    Py_Finalize();

    t1.join();
    return 0;
}