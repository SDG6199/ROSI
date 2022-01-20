#include <ros.h>
#include <std_msgs/Byte.h>

ros::NodeHandle nh;
std_msgs::Byte msg;
ros::Publisher pole1_tag_pub("pole1_sensor",&msg);

const int trackingPin = 9;
int val, tag_value=0;

unsigned long old_time,tag_old_time=0;
unsigned long react_sec=750;  //ms
unsigned long maintain_sec=5000;
unsigned long ir_old_time=4251;  //@ can change

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  nh.initNode();
  nh.advertise(pole1_tag_pub);
  pinMode(trackingPin, INPUT); // set trackingPin as INPUT
}

void loop()
{
  val = digitalRead(trackingPin); // read the value of tracking module

  if(val==1 && (millis()-tag_old_time)>maintain_sec)
  {
    nh.logwarn("ir not detect");
    tag_value=0;
    ir_old_time=millis();
  }
  else if(val==0)
  {
    nh.logwarn("ir detect");
    tag_old_time=millis(); 
  }
  if(millis()>ir_old_time+react_sec)
  {
    tag_value=1;     
  }
    
  if(millis()-old_time>=1)
  {
    msg.data=tag_value;
    pole1_tag_pub.publish(&msg);
    old_time=millis();
  }
  delay(25);
  nh.spinOnce();
}
