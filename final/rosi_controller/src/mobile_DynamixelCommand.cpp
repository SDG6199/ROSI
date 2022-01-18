//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <darknet_ros_msgs/BoundingBoxes.h> 
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <python2.7/Python.h>   
#include <thread>      

int pole_tag=0;
double motor_velocity=200;
bool is_car=false;
std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> detected_bboxes_; //vector로 변환 
gb_visual_detection_3d_msgs::BoundingBox3d car_box;
double pix_x,pix_y,pix_z; 

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
void detect_box(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr& msg)
{
    detected_bboxes_ = msg->bounding_boxes;  //BoundingBox3d[]. 즉, array형태이다.
    int size =detected_bboxes_.size();
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
void call_motor(dynamixel_workbench_msgs::DynamixelCommand& srv, ros::ServiceClient& client, const double& velocity)
{
    srv.request.value=velocity;
    srv.request.id = 1;
    client.call(srv);
    
    srv.request.value=-velocity;
    srv.request.id = 2;
    client.call(srv);
}
void thread_present_pos(PyObject* Name)
{
    PyObject* Module = PyImport_Import(Name);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile");
    ros::NodeHandle n; 
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_mobile/dynamixel_command");
    ros::Subscriber sub1 = n.subscribe("pole1_sensor", 1000, pole1_callback);
    ros::Subscriber sub2 = n.subscribe("pole2_sensor", 1000, pole2_callback);
    ros::Subscriber sub3 = n.subscribe("pole3_sensor", 1000, pole3_callback);
    ros::Subscriber sub4 = n.subscribe("pole4_sensor", 1000, pole4_callback);
    ros::Subscriber car_box_sub = n.subscribe("/darknet_ros_3d/bounding_boxes", 1000, detect_box);     
    ros::Publisher save_pub = n.advertise<std_msgs::Byte>("/image_save_tag", 1000);

    dynamixel_workbench_msgs::DynamixelCommand srv;

    ros::Rate rate(10);
    ros::Duration turn_time= ros::Duration(5);
    std_msgs::Byte savetag;
    ros::Time prev_time=ros::Time::now();
    ros::Time current_time=ros::Time::now();
    ros::Time isnot_car_time=ros::Time::now();
    bool flag_default=true;

    std::string item_command = "";
    std::string item_addr = "Goal_Velocity";
    
    srv.request.command = item_command;
    srv.request.addr_name = item_addr;
    srv.request.value=0; 

    PyObject *pName,*pValue,*pModule,*pFunc;
    PyObject* pArgs;

    Py_Initialize();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(\"/home/sdg/catkin_ws/src/rosi_controller/script\")");
    pName = PyUnicode_FromString("velocity_mode_presPos");  //PyObject 생성.
    //pModule = PyImport_Import(pName); //생성한 PyObject pName을 import한다.
    //    pFunc = PyObject_GetAttrString(pModule, "test"); 
    //    pFunc = PyObject_GetAttrString(pModule, "get_present_pos"); //실행할 함수를 PyObject에 전달.
    //    pValue = PyObject_CallObject(pFunc, NULL); //매개변수(pArgs)를 전달하며 pFunc를 호출한다. 현재 매개변수가 NULL인 경우이다. return값을 받으려면 PyLong_AsLong을 써야한다.

    std::thread thread1(thread_present_pos,pName);
    thread1.detach();

    while(ros::ok())
    {
        printf("hi2");
        if(pole_tag==1)
        {
           //출발시 전진, 도착시 정지   //1@
        }
        else if(pole_tag==2)
        {
            call_motor(srv,client,0);
            turn_time.sleep();  //5초 대기
        }
        else if(pole_tag==3)
        {
            //도착시 정지.
        }
        else if(pole_tag==4)
        {
            //도착시 정지.
        }
        else
        {
            //default
            call_motor(srv,client,motor_velocity);

            if(is_car==true)
            {
                current_time=ros::Time::now();
                while(current_time-isnot_car_time<ros::Duration(2))
                {
                    current_time=ros::Time::now();
                    call_motor(srv,client,motor_velocity/2.0);
                    rate.sleep();
                    ros::spinOnce();
                }
                
                /*

                if(차를 처음봤는지)
                {
                    // store pixel coordinate(car)
                    pix_car_x=(car_box.xmax-car_box.xmin)/2.0;
                    pix_car_y=(car_box.ymax-car_box.ymin)/2.0;
                    pix_car_z=(car_box.zmax-car_box.zmin)/2.0;
                    //prev_pix_car_position(vector)에 pix_car_x,pix_car_y,pix_car_z.

                    //store present position
                    encoder=(present position)
                    prev_distance=(encoder-prev_encoder)*바퀴직경*pi       //2@
                } 
                else
                {
                    // store pixel coordinate(car)
                    pix_car_x=(car_box.xmax-car_box.xmin)/2.0;
                    pix_car_y=(car_box.ymax-car_box.ymin)/2.0;
                    pix_car_z=(car_box.zmax-car_box.zmin)/2.0;
                    //pix_car_position(vector)에 pix_car_x,pix_car_y,pix_car_z.

                    //store present position
                    encoder=(present position)
                    distance=(encoder-prev_encoder)*바퀴직경*pi

                    if(distance==prev_distance && pix_car_position==prev_pix_car_position && 30초 지남)  //2@
                    {
                        Illegal_parking_data.txt에 차번호 저장 후 display.
                    }
                }             
                */

                savetag.data=1;
                save_pub.publish(savetag);
            }
            else
            {
                prev_time=ros::Time::now();

                if(prev_time-current_time>ros::Duration(1))
                {
                    isnot_car_time=ros::Time::now();
                    savetag.data=0;
                    save_pub.publish(savetag);
                    //ROS_INFO("isnt_car");
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

            if(pix_human_position이 횡단불가영역에 포함된다)  //3@
            {
                부저울림토픽 publish.
            }
            */
        }

        rate.sleep();
        ros::spinOnce();
    }
    
    Py_Finalize();
    return 0;
}
