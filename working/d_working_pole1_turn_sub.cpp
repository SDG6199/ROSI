//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>

void angle_cb(const std_msgs::Byte::ConstPtr& ms)
{
    if (msg->data==0)
    {
        motor_angle=0;
    }
    if (msg->data==1)
    {
        motor_angle=1024;
    }
    if (msg->data==2)
    {
        motor_angle=2048;
    }
    if (msg->data==3)
    {
        motor_angle=3072;
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pole1_turn_sub");
    ros::NodeHandle n; 
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_pole1/dynamixel_command");
    ros::Subscriber pole_angle_sub = n.subscribe("pole1/pole_angle", 1000, angle_cb);

    dynamixel_workbench_msgs::DynamixelCommand srv;
    int motor_angle=2048;   //0~4095

    std::string item_command = "";
    std::string item_addr = "Goal_Position";
    srv.request.command = item_command;
    srv.request.id = 1;
    srv.request.addr_name = item_addr;
    srv.request.value=motor_angle; 

    ros::Rate rate(10);
    while(ros::ok())
    {
        srv.request.value=motor_angle;
        client.call(srv);

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
