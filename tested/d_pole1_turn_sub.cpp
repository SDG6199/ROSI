//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>

int pole1_tag=0;
int motor_angle=3072;   //0~4095
int count=0;
int flag=1;

void pole1_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if (msg->data==1) // pole1 tag!
    {
        pole1_tag=1;
    }
    else
    {
        pole1_tag=0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pole1_turn_sub");
    ros::NodeHandle n;
    ros::Time current_time=ros::Time::now();
    ros::Time prev_time=ros::Time::now();
    
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
    dynamixel_workbench_msgs::DynamixelCommand srv;
    ros::Rate rate(1);
    //ros::Duration back_time= ros::Duration(4);

    std::string item_command = "";
    std::string item_addr = "Goal_Position";
    
    srv.request.command = item_command;
    srv.request.id = 1;
    srv.request.addr_name = item_addr;

    srv.request.value=motor_angle; 
    if (client.call(srv))
    {
        ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
        ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
    }
    else
    {
        ROS_ERROR("Failed to call dynamixel_command");
    }

    while(ros::ok())
    {
        ros::Subscriber sub1 = n.subscribe("is_pole1_tag", 1000, pole1_callback);

        if(pole1_tag==1)
        {
            current_time=ros::Time::now();
            ros::Duration d= current_time-prev_time;
            if(d>=ros::Duration(10))
            {
                flag=1;
            }
            if(flag==1)
            {
                flag=0;
                count++;
            }
            else
            {
              if(count%2==1)
              {
                  srv.request.value=2048;  //1024만큼 회전.
  
                  if (client.call(srv))
                  {
                      ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                      ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                  }
                  else
                  {
                      ROS_ERROR("Failed to call dynamixel_command");
                  }
              }
              else if(count%2==0)
              {
                  srv.request.value=motor_angle;		//복귀.
  
                  if (client.call(srv))
                  {
                      ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                      ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                  }
                  else
                  {
                      ROS_ERROR("Failed to call dynamixel_command");
                  }
              }
            }
        }
        else
        {
            flag=1;
            prev_time=ros::Time::now(); 
        }
     
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
