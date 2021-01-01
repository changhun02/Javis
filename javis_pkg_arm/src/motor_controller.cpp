#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_workbench_msgs/Javis_msgs.h>
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
#define ID7                             7
#define ID6                             6
#define ID5                             5
#define ID4                             4
#define ID3                             3
#define ID2                             2
//#define ID1                             1
//#define ID0                             0

#define EX_CENTER                       2048
#define MX_CENTER                       2048
#define RX_CENTER                       512
//#define AX_CENTER                       512

#define DEVICENAME                     "/dev/ttyUSB1"
//#define DEVICENAME_GRAB                "/dev/ttyUSB0"
#define BAUDRATE                        1000000

#define PI                              3.141592

#define PULSE_PUR_DEGREE_MX             11.378
#define PULSE_PUR_DEGREE_EX             16.319
#define PULSE_PUR_DEGREE_RX             3.41
//#define PULSE_PUR_DEGREE_AX             3.41

using namespace std;
using namespace dynamixel;
using namespace Eigen;

double center_position[6] = {MX_CENTER, EX_CENTER, EX_CENTER, RX_CENTER, RX_CENTER, RX_CENTER};
double angle[6] = { 0.0 , 90.0, -90.0, 0.0, 0.0, 0.0};
double pulse[6] = { 0 ,};

void maniCallback(const dynamixel_workbench_msgs::Javis_msgs& msg)
{
  for(int i = 0 ; i < 6 ; i++){
    angle[i] = msg.angle[i] * (180/PI);
    cout << angle[i] << endl;
  }
  cout << "###########################" << endl;
}
int main(int argc, char **argv)
{
  PortHandler *pohandler0 = PortHandler::getPortHandler(DEVICENAME);
  PacketHandler *pkhandler0 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);


  if(pohandler0->openPort()){
      if(pohandler0->setBaudRate(BAUDRATE))
        printf("Success!\n");
  }

//  if(pohandler0->openPort() & pohandler1->openPort()){
//    if(pohandler0->setBaudRate(BAUDRATE) & pohandler1->setBaudRate(BAUDRATE))
//      printf("Success!\n");
//  }
  else{
    printf("Fail!\n");
    return 0;
  }

  //RX-64
  pkhandler0->write1ByteTxRx(pohandler0, ID2, ADDR_TORQUE_ENABLE, 1);
  pkhandler0->write1ByteTxRx(pohandler0, ID3, ADDR_TORQUE_ENABLE, 1);
  pkhandler0->write1ByteTxRx(pohandler0, ID4, ADDR_TORQUE_ENABLE, 1);
  //EX-106
  pkhandler0->write1ByteTxRx(pohandler0, ID5, ADDR_TORQUE_ENABLE, 1);
  pkhandler0->write1ByteTxRx(pohandler0, ID6, ADDR_TORQUE_ENABLE, 1);
  //MX-106
  pkhandler0->write1ByteTxRx(pohandler0, ID7, ADDR_TORQUE_ENABLE, 1);

  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("manipulator", 1000, maniCallback);
  ros::Rate rate(25);

  while(ros::ok()){

    pulse[0] = 2048 + (angle[0] * PULSE_PUR_DEGREE_MX);
    pkhandler0->write4ByteTxRx(pohandler0, 7, ADDR_GOAL_POSITION, pulse[0]);

    pulse[1] = 2048 - (angle[1] * PULSE_PUR_DEGREE_EX);
    pkhandler0->write4ByteTxRx(pohandler0, 6, ADDR_GOAL_POSITION, pulse[1]);

    pulse[2] = 2048 + (angle[2] * PULSE_PUR_DEGREE_EX);
    pkhandler0->write4ByteTxRx(pohandler0, 5, ADDR_GOAL_POSITION, pulse[2]);

    pulse[3] = 512 - (angle[3] * PULSE_PUR_DEGREE_RX);
    pkhandler0->write4ByteTxRx(pohandler0, 4, ADDR_GOAL_POSITION, pulse[3]);

    pulse[4] = 512 + (angle[4] * PULSE_PUR_DEGREE_RX);
    pkhandler0->write4ByteTxRx(pohandler0, 3, ADDR_GOAL_POSITION, pulse[4]);

    pulse[5] = 512 + (angle[5] * PULSE_PUR_DEGREE_RX);
    pkhandler0->write4ByteTxRx(pohandler0, 2, ADDR_GOAL_POSITION, pulse[5]);

    // angle control
    //pkhandler1->write4ByteTxRx(pohandler1, ID0, ADDR_GOAL_POSITION, 512);
    //pkhandler1->write4ByteTxRx(pohandler1, ID1, ADDR_GOAL_POSITION, 512);

    ros::spinOnce();
    rate.sleep();
  }
  pohandler0->closePort();
}
