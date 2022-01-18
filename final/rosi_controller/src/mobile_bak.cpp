//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
//#include "std_msgs/String.h"
//#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <darknet_ros_msgs/BoundingBoxes.h> 
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <python2.7/Python.h>   
//#include <thread>
#include <boost/thread/thread.hpp>  //use thread in ros

#define pi 3.141592653589

int pole_tag=0;  
int pole_tag_flag=1;
int motor_velocity=-200;
double diameter_wheel= 0.0326;    //m  @
double to_rad_coef=pi/2048;
bool is_car=false;
bool is_plastic_person=false;
bool is_pix_in_tor=false;
int path_vec_update_flag=0;
int pole_rel=0;

std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> detected_bboxes_; //vector로 변환 
std::vector<std::string> detected_bboxes_class_;

gb_visual_detection_3d_msgs::BoundingBox3d car_box;
gb_visual_detection_3d_msgs::BoundingBox3d plastic_man_box;
double pix_x,pix_y,pix_z; 
int state_store_flag=1;
int stay_for_store_flag=0;
std::vector<double> pix_car_position(3,0);
std::vector<double> toler_pix_car_position(3,0.5); //수정가능@
std::vector<double> prev_pix_car_position(3,0);
std::vector<double> tor_plus_vector(3,0);
std::vector<double> tor_minus_vector(3,0);
std::vector<int> path_vec;
std::array<int,2> pole_num_arr;
std::array<int,2> pole_angle_arr;


double pos,prev_pos,prev_pos_rad,pos_rad,distance,prev_distance;
double toler_distance=0.1;

std_msgs::Byte savetag;
std_msgs::Byte direction_roi;
std_msgs::Byte pole_angle;

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
//        car_box.xmax=0; car_box.xmin=0;
//        car_box.ymax=0; car_box.ymin=0;  
//        car_box.zmax=0; car_box.zmin=0; 
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
    ros::Rate loop_rate(30);
    while(ros::ok())
    {   
        save_pub.publish(savetag);
        loop_rate.sleep();
    }
}

void thread_pole_angle()
{
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
    ros::Publisher pole1_angle_pub = n->advertise<std_msgs::Byte>("pole1/pole_angle", 1000);
    ros::Publisher pole2_angle_pub = n->advertise<std_msgs::Byte>("pole2/pole_angle", 1000);
    ros::Publisher pole3_angle_pub = n->advertise<std_msgs::Byte>("pole3/pole_angle", 1000);
    ros::Publisher pole4_angle_pub = n->advertise<std_msgs::Byte>("pole4/pole_angle", 1000);
    ros::Publisher direction_roi_pub = n->advertise<std_msgs::Byte>("/direction_roi", 1000);
    
    ros::Rate loop_rate(30);
    while(ros::ok())
    {   
        for(int i=0; i<2; i++)
        {
            pole_angle.data=pole_angle_arr[i];

            switch(pole_num_arr[i])  
            {
                case 2:   
                    pole1_angle_pub.publish(pole_angle);
                    break;
                case 5:
                    pole2_angle_pub.publish(pole_angle);
                    break;
                case 4: 
                    pole3_angle_pub.publish(pole_angle);
                    break;
                case 6:
                    pole4_angle_pub.publish(pole_angle);
                    break;   
            }        
        }
        direction_roi_pub.publish(direction_roi);
        loop_rate.sleep();
    } 
}
void thread_pole_num()  //path_vec의 2요소씩 검사.
{   
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();

    int path_index=0;

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        if(path_vec_update_flag==1)
        {
            if(path_index==path_vec.size()-1)
            {
                path_index=0;
            }
            pole_num_arr[0]=path_vec[path_index];
            pole_num_arr[1]=path_vec[path_index+1];

            pole_rel=pole_num_arr[1]-pole_num_arr[0];
            switch(pole_rel)
            {
                case -1:    //left
                    pole_angle_arr[0]=3;
                    pole_angle_arr[1]=3;
                    direction_roi.data=1;
                    break;
                case +1:    //right
                    pole_angle_arr[0]=1;
                    pole_angle_arr[1]=1;
                    direction_roi.data=2;
                    break;
                case +3:    //up
                    pole_angle_arr[0]=0;
                    pole_angle_arr[1]=0;
                    direction_roi.data=0;
                    break;
                case -3:    //down
                    pole_angle_arr[0]=2;
                    pole_angle_arr[1]=2;
                    direction_roi.data=0;
                    break;
            }
            path_index++;
            path_vec_update_flag=0;
        }
        loop_rate.sleep();
    }
}
int main(int argc, char **argv)
{
    if(argc>2)
    {
        for(int i=1; i<argc; i++)
        {
            path_vec.push_back(atoi(argv[i]));
            //std::cout << path_vec[i-1] << std::endl;
        }
    }
    else
    {
        ROS_ERROR("Enter a number of points 2 or more. \n\n");
    }
    ros::init(argc, argv, "mobile");
    //ros::NodeHandle n; 
    ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();

    ros::Subscriber sub1 = n->subscribe("pole1_sensor", 1000, pole1_callback);
    ros::Subscriber sub2 = n->subscribe("pole2_sensor", 1000, pole2_callback);
    ros::Subscriber sub3 = n->subscribe("pole3_sensor", 1000, pole3_callback);
    ros::Subscriber sub4 = n->subscribe("pole4_sensor", 1000, pole4_callback);
    ros::Subscriber car_box_sub = n->subscribe("/darknet_ros_3d/bounding_boxes", 1000, detect_box);     

    ros::Rate rate(30);
    
    ros::Duration turn_time= ros::Duration(10); //@
    ros::Duration slow_time= ros::Duration(7);
    ros::Duration capture_time= ros::Duration(2);

    ros::Time temp_time=ros::Time::now();
    ros::Time is_car_time=ros::Time::now();
    ros::Time isnot_car_time=ros::Time::now();
    ros::Time stored_time=ros::Time::now();
    ros::Time under_sensor_time=ros::Time::now();

    PyObject *pName,*pValue,*pModule,*SET_VEL, *GET_STATUS, *CLEAR;
    PyObject *pArgs;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"/home/jelly/catkin_ws/src/rosi_controller/script\")");
    pArgs = PyTuple_New(2);

    pName = PyUnicode_FromString("velocity_mode_read_status");  //PyObject 생성.
    pModule = PyImport_Import(pName); //생성한 PyObject pName을 import한다.

    SET_VEL = PyObject_GetAttrString(pModule, "set_vel"); //실행할 함수를 PyObject에 전달.
    GET_STATUS = PyObject_GetAttrString(pModule, "get_status");
    CLEAR = PyObject_GetAttrString(pModule, "clear");
    //pValue = PyObject_CallObject(pFunc, NULL); //매개변수(pArgs)를 전달하며 pFunc를 호출한다. 현재 매개변수가 NULL인 경우이다. return값을 받으려면 PyLong_AsLong을 써야한다.

    //std::thread t1(thread_save_mode);
    boost::thread t1(thread_save_mode);
    boost::thread t2(thread_pole_angle);
    boost::thread t3(thread_pole_num);
    
    
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
        if(pole_tag==1 && pole_tag_flag==1)
        {
	        ROS_INFO("pole1_taged! \n");
            under_sensor_time=ros::Time::now();
            path_vec_update_flag=1;

            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //10초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;

            pole_tag_flag=0;
        }
        else if(pole_tag==2 && pole_tag_flag==1)  //도착시 정지.
        {
	        ROS_INFO("pole2_taged! \n");
            under_sensor_time=ros::Time::now();
            path_vec_update_flag=1;

            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //10초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;

            pole_tag_flag=0;
        }
        else if(pole_tag==3 && pole_tag_flag==1)  //도착시 정지.
        {
	        ROS_INFO("pole3_taged! \n");
            under_sensor_time=ros::Time::now();
            path_vec_update_flag=1;

            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //10초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;

            pole_tag_flag=0;
        }
        else if(pole_tag==4 && pole_tag_flag==1)  //도착시 정지.
        {
	        ROS_INFO("pole4_taged! \n");
            under_sensor_time=ros::Time::now();
            path_vec_update_flag=1;

            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
            PyObject_CallObject(SET_VEL, pArgs); 
            turn_time.sleep();  //10초 대기
            pValue = PyObject_CallObject(GET_STATUS, NULL);
            prev_pos = PyLong_AsLong(pValue);
            prev_pos_rad = prev_pos*to_rad_coef;

            pole_tag_flag=0;
        }
        else
        {
            //default 
            PyTuple_SetItem(pArgs, 0, PyLong_FromLong(motor_velocity)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(-motor_velocity));
            PyObject_CallObject(SET_VEL, pArgs);

            if(ros::Time::now()-under_sensor_time>ros::Duration(18))
            {
                pole_tag_flag=1;  
            }

            if(is_car==true && (pole_rel==1 || pole_rel==-1))
            {
                is_car_time=ros::Time::now(); 

                while(is_car_time-isnot_car_time<ros::Duration(9))
                {
                    PyTuple_SetItem(pArgs, 0, PyLong_FromLong(motor_velocity/2)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(-motor_velocity/2));
                    PyObject_CallObject(SET_VEL, pArgs);
                    slow_time.sleep();      //7초 대기 
                    PyTuple_SetItem(pArgs, 0, PyLong_FromLong(0)); PyTuple_SetItem(pArgs, 1, PyLong_FromLong(0));
                    PyObject_CallObject(SET_VEL, pArgs);
                    capture_time.sleep();   //2초 대기 
                    
                    //store previous pixel coordinate
                    pix_car_position[0]=(car_box.xmax+car_box.xmin)/2.0;   
                    pix_car_position[1]=(car_box.ymax+car_box.ymin)/2.0;
                    pix_car_position[2]=(car_box.zmax+car_box.zmin)/2.0;
                
                    is_car_time=ros::Time::now();
                    rate.sleep();
                    ros::spinOnce();
                }

                if(state_store_flag==1)  //store state.
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
                else   //비교
                {
                    //store present position
                    pValue = PyObject_CallObject(GET_STATUS, NULL);
                    pos = PyLong_AsLong(pValue);
                    pos_rad = pos*to_rad_coef;
                    distance=(pos_rad-prev_pos_rad)*diameter_wheel*pi;    //m
                    
                    is_pix_in_tor=pix_in_tor(pix_car_position,prev_pix_car_position,toler_pix_car_position);
                    
                    /*if(prev_distance-toler_distance<distance<prev_distance+toler_distance && \
                    is_pix_in_tor && (ros::Time::now()-stored_time)>ros::Duration(5))
                    {*/
                    stored_time=ros::Time::now();
                    prev_pix_car_position=pix_car_position;
                    prev_distance=distance;

                    savetag.data=2;  
                    stay_for_store_flag=1; 
                    is_pix_in_tor=false; 
                }  
            }
            else
            {   
                temp_time=ros::Time::now();

                if(temp_time-is_car_time>ros::Duration(1))
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
        }

        rate.sleep();
        ros::spinOnce();
    }
    
    PyObject_CallObject(CLEAR, NULL);
    Py_Finalize();

    t1.join();
    t2.join();
    t3.join();
    return 0;
}
