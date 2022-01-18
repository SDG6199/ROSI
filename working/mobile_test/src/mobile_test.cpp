//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>


void angle_cb(const std_msgs::Byte::ConstPtr& msg);
int motor_Velocity=160;   //0~4095
int count=0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_test");
    ros::NodeHandle n; 
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    ros::Duration go_time= ros::Duration(10);
    ros::Duration stop_time= ros::Duration(3);
    dynamixel_workbench_msgs::DynamixelCommand srv;

    std::string item_command = "";
    std::string item_addr = "Goal_Velocity";
    srv.request.command = item_command;
    srv.request.id = 1;
    srv.request.addr_name = item_addr;
    srv.request.value=motor_Velocity; 

    ros::Rate rate(10);
    while(ros::ok())
    {

        srv.request.value=160;
        client.call(srv);
        go_time.sleep();

        srv.request.value=0;
        client.call(srv);
        stop_time.sleep();

        ros::spinOnce();
    }
    return 0;
}
