//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>

int pole2_tag=0;
int motor_angle=3072;   //0~4095
int turn_count=0;
int flag=1;

void pole2_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1){pole2_tag=1;}
    else{pole2_tag=0;}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pole2_turn_sub");
    ros::NodeHandle n; 
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    dynamixel_workbench_msgs::DynamixelCommand srv;
    ros::Rate rate(1);
    //ros::Duration back_time= ros::Duration(4);
    std::string item_command = "";
    std::string item_addr = "Goal_Position";
    ros::Duration turn_time= ros::Duration(7);
    srv.request.command = item_command;
    srv.request.id = 2;
    srv.request.addr_name = item_addr;
    srv.request.value=motor_angle; 

    while(ros::ok())
    {
        ros::Subscriber sub1 = n.subscribe("pole2_sensor", 1000, pole2_callback);
        if(pole2_tag==1)
        {
            if(turn_count%2==0)
            {
                srv.request.value=2048;
                client.call(srv);
            }
            else
            {
                srv.request.value=motor_angle;
                client.call(srv);
            }
            turn_count++;
            ROS_INFO("turn_count=%d",turn_count);
            turn_time.sleep();
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
