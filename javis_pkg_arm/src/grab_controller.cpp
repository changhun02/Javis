#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_msgs/javis_grab.h>
#include <eigen3/Eigen/Dense>

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>

//Grab left: 0, right: 1
#define ADDR_TORQUE_ENABLE              24
#define ADDR_GOAL_POSITION              30
#define ADDR_PRESENT_POSITION           64
#define PROTOCOL_VERSION                1.0
#define ID1                             1
#define ID0                             0

#define AX_CENTER                       512

#define DEVICENAME_GRAB                "/dev/ttyUSB0"
#define BAUDRATE                        1000000

#define PI                              3.141592

#define PULSE_PUR_DEGREE_AX             3.41

using namespace std;
using namespace dynamixel;
using namespace Eigen;
bool g_open;

void grabCallback(const dynamixel_workbench_msgs::javis_grab& msg)
{
  if(msg.grab)
    g_open = true;
  else
    g_open = false;
}

int main(int argc, char **argv)
{
  PortHandler *pohandler1 = PortHandler::getPortHandler(DEVICENAME_GRAB);
  PacketHandler *pkhandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(pohandler1->openPort()){
      if(pohandler1->setBaudRate(BAUDRATE))
        printf("Success!\n");
  }
  else{
    printf("Fail!\n");
    return 0;
  }

  //AX-12
  pkhandler1->write1ByteTxRx(pohandler1, ID0, ADDR_TORQUE_ENABLE, 1);
  pkhandler1->write1ByteTxRx(pohandler1, ID1, ADDR_TORQUE_ENABLE, 1);

  ros::init(argc, argv, "grab_controller");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("grab", 1000, grabCallback);

  ros::Rate rate(20);
  while(ros::ok()){
    if(g_open){
      pkhandler1->write4ByteTxRx(pohandler1, ID0, ADDR_GOAL_POSITION, 724);
      pkhandler1->write4ByteTxRx(pohandler1, ID1, ADDR_GOAL_POSITION, 300);
      std::cout << "Open!" << std::endl;
    }
    else{
      pkhandler1->write4ByteTxRx(pohandler1, ID0, ADDR_GOAL_POSITION, 512);
      pkhandler1->write4ByteTxRx(pohandler1, ID1, ADDR_GOAL_POSITION, 512);
      std::cout << "Close!" << std::endl;
    }
    ros::spinOnce();
    rate.sleep();
  }
  pohandler1 -> closePort();
}
