#include "ros/ros.h"
#include "dynamixel_workbench_msgs/javis_des.h"
#include "dynamixel_workbench_msgs/javis_grab.h"

using namespace std;
int main(int argc, char **argv)
{
  int grab;
  double des[3] = {0.36, 0.101, 0.21};
  ros::init(argc, argv, "des_pub");
  ros::NodeHandle nh;

  ros::Publisher des_pub = nh.advertise<dynamixel_workbench_msgs::javis_des>("destination", 1000);
  ros::Publisher grab_pub = nh.advertise<dynamixel_workbench_msgs::javis_grab>("grab", 1000);
  dynamixel_workbench_msgs::javis_des msg;
  dynamixel_workbench_msgs::javis_grab msg_;

  for(int i = 0 ; i < 3 ; i++){
    msg.des[i] = des[i];
  }
  des_pub.publish(msg);

  while (ros::ok())
  {
    cout << "grab : ";
    cin >> grab;
    if(grab == 1){
      msg_.grab = true;
      cout << "True!" << endl;
    }
    else{
      msg_.grab = false;
      cout << "false!" << endl;
    }

    grab_pub.publish(msg_);

    cout << "x : ";
    cin >> des[0];
    msg.des[0] = des[0];

    cout << "y : ";
    cin >> des[1];
    msg.des[1] = des[1];

    cout << "z : ";
    cin >> des[2];
    msg.des[2] = des[2];

    des_pub.publish(msg);
    cout << "Done!" << endl;
    ros::spinOnce();
  }

  return 0;
}
