#include "ros/ros.h"
#include <dynamixel_workbench_msgs/Javis_msgs.h>
#include <dynamixel_workbench_msgs/javis_grab.h>
#include <dynamixel_workbench_msgs/javis_des.h>

void chatterCallback(const dynamixel_workbench_msgs::Javis_msgs& msg)
{
  std::cout << msg.angle[0] << ", " << msg.angle[1] << ", "<< msg.angle[2] << ", "<< msg.angle[3] << ", "<< msg.angle[4] << ", "<< msg.angle[5] << std::endl;
}

void chatterCallback1(const dynamixel_workbench_msgs::javis_des& msg)
{
  std::cout << msg.des[0] << ", " << msg.des[1] << ", "<< msg.des[2] << std::endl;
}
void chatterCallback2(const dynamixel_workbench_msgs::javis_grab& msg)
{
  std::cout << msg.grab << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("manipulator", 1000, chatterCallback);
  ros::Subscriber sub1 = nh.subscribe("destination", 1000, chatterCallback1);
  ros::Subscriber sub2 = nh.subscribe("grab", 1000, chatterCallback2);
  ros::spin();

  return 0;
}
