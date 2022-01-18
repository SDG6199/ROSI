//initialization related to ros 
#include "ros/ros.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Int32.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include <cstdlib>

int pole1_tag, pole2_tag, narangd_detected, pole1_tag_count=0;
int motor_speed=160; //0~265
int pole1_tag_flag_odd, pole1_tag_flag_even=0;
int flag = 1;

void pole1_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if(msg->data==1) // pole1 tag!
    {
        pole1_tag=1;
    }
    else
    {
        pole1_tag=0;
    }
}

void pole2_callback(const std_msgs::Byte::ConstPtr& msg)
{
    if(msg->data==1) // pole2 tag!
    {
        pole2_tag=1;
    }
    else
    {
        pole2_tag=0;
    }
}

void narangd_callback(const std_msgs::Int32::ConstPtr& msg)
{
    if (msg->data==1) // narangd detected!
    {
        narangd_detected=1;
    }
    if (msg->data==0)
    {
        narangd_detected=0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pole_tag_sub");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_mobile/dynamixel_command");
    ros::Time current_time = ros::Time::now();
    ros::Time prev_time=ros::Time::now();
    ros::Time current_time_odd = ros::Time::now();  
    ros::Time current_time_even = ros::Time::now(); 
       
    dynamixel_workbench_msgs::DynamixelCommand srv;
    ros::Rate rate(1);
    ros::Duration turn_time= ros::Duration(2);
    ros::Duration delay_time= ros::Duration(1.5);
    std::string item_command = "";
    std::string item_addr = "Goal_Velocity";
    
    srv.request.command = item_command;
    srv.request.addr_name = item_addr;
    
    while(ros::ok())
    {
        ros::Subscriber sub1 = n.subscribe("is_pole1_tag", 1000, pole1_callback);
        ros::Subscriber sub2 = n.subscribe("is_pole2_tag", 1000, pole2_callback);
        ros::Subscriber sub3 = n.subscribe("/darknet_3d/is_narangd_detected", 1000, narangd_callback);

          
        if(narangd_detected==1)
        {  
            //---object captured!
            srv.request.value=0;

            srv.request.id = 1;
            if (client.call(srv))
            {
                ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
            }
            else
            {
                ROS_ERROR("Failed to call dynamixel_command");
            }
            srv.request.id = 2;
            if (client.call(srv))
            {
                ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
            }
            else
            {
                ROS_ERROR("Failed to call dynamixel_command");
            }
            delay_time.sleep();
            flag=1;
            //---
        }
        else
        {
            if(flag==1)
            {
              if(pole1_tag==1)
              {
                    ROS_ERROR("pole1 tag!!");
                    flag=0;
                    pole1_tag_count++;
                    if(pole1_tag_count%2==1)
                    {
                        pole1_tag_flag_odd=1;
                        pole1_tag_flag_even=0;
                    }
                    else
                    {
                        pole1_tag_flag_even=1;
                        pole1_tag_flag_odd=0;
                    }
                    
                    srv.request.value=0;
    
                    srv.request.id = 1;
                    if (client.call(srv))
                    {
                        ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                        ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call dynamixel_command");
                    }
                    srv.request.id = 2;
                    if (client.call(srv))
                    {
                        ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                        ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call dynamixel_command");
                    }
                    turn_time.sleep();              //2second  
                    prev_time = ros::Time::now(); 
                  //---
              }
            }
            if(pole1_tag_flag_odd==1)
            {          
                ROS_ERROR("pole1_odd_back!!");
                pole1_tag_flag_even=0;
                current_time_odd = ros::Time::now(); 
                if(current_time_odd-prev_time>=ros::Duration(7))
                {
                    flag=1;
                }
                
                //backward moving
                srv.request.value=-motor_speed;
                srv.request.id = 1;
                if (client.call(srv))
                {
                    ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                    ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                }
                else
                {
                    ROS_ERROR("Failed to call dynamixel_command");
                }
                srv.request.value=motor_speed;
                srv.request.id = 2;
                if (client.call(srv))
                {
                    ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                    ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                }
                else
                {
                    ROS_ERROR("Failed to call dynamixel_command");
                }
                if(pole2_tag==1)
                {
                    pole1_tag_flag_odd=0;
                }
                //---
            }
            else if(pole1_tag_flag_even==1)
            {
                ROS_ERROR("pole1_even_back!!");
                pole1_tag_flag_odd=0;
                current_time_even = ros::Time::now();
                if(current_time_even-prev_time>=ros::Duration(7))
                {
                    flag=1;
                }
                
                //backward moving(for 5 seconds.)
                current_time = ros::Time::now();
                if(current_time-prev_time<=ros::Duration(9))
                {

                    srv.request.value=-motor_speed;
                    srv.request.id = 1;
                    if (client.call(srv))
                    {
                        ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                        ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                    }
                    else
                    {
                        ROS_ERROR("Failed to call dynamixel_command");
                    }
                    srv.request.value=motor_speed;
                    srv.request.id = 2;
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
                else
                {
                    pole1_tag_flag_even=0;
                }
                  //---
            }
            else
            {
                ROS_ERROR("default advance!!");
                flag=1;
                prev_time = ros::Time::now(); 
                //---forward moving(default)
                srv.request.value=motor_speed; //; 
                srv.request.id = 1;
                if (client.call(srv))
                {
                    ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                    ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                }
                else
                {
                    ROS_ERROR("Failed to call dynamixel_command");
                }
                srv.request.value=-motor_speed; //; 
                srv.request.id = 2;
                if (client.call(srv))
                {
                    ROS_INFO("send ID and Position Value : %ld, %ld", (uint8_t)srv.request.id, (int32_t)srv.request.value);
                    ROS_INFO("receive result : %ld", (bool)srv.response.comm_result);
                }
                else
                {
                    ROS_ERROR("Failed to call dynamixel_command");
                }
                //---
             }   
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
