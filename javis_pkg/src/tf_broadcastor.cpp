#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14192
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster, broadcaster1, broadcaster2;
  tf::Transform transform, transform1, transform2;
  tf::Quaternion q, q1, q2;

  q.setRPY(0, 0, 0);
  q1.setRPY(3.141592, 0, 0);
  q2.setRPY(0, 0, 0);

  while (n.ok()) {

    //transform2.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    //transform2.setRotation(q2);
    //broadcaster2.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint","base_link"));

    transform.setOrigin(tf::Vector3(0.0, 0.0, 1.0));
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","camera_link"));

    transform1.setOrigin(tf::Vector3(0.25, 0.0, 0.05));
    transform1.setRotation(q1);
    broadcaster1.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "base_link","laser"));

    transform2.setOrigin(tf::Vector3(0.12, 0.0, 0.25));
    transform2.setRotation(q2);
    broadcaster2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "base_link","base_arm_link"));
    r.sleep();
  }
}
/*
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
   broadcaster.sendTransform(
     tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0 , 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "laser"));
    r.sleep();
  }
}
*/
