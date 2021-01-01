/*#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){

  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

}
int main(int argc, char **argv){

  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle n;//create a handle node

  ros::Subscriber sub = n.subscribe("/robot_pose_ekf/odom_combined",1000, chatterCallback);
  //subscriber lister the msgs of given topic
  ros::spin();//run it until you press ctrl+c
  return 0;
}
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define PI 3.141592
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_cameralink");
  ros::NodeHandle n;

  ros::Rate r(10);

  tf::TransformBroadcaster broadcaster;
  tf::Transform transform;
  tf::Quaternion q, q1, q2;

  q.setRPY(0, 0, 0);

  while (n.ok()) {

    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.3));
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","camera_link"));

    r.sleep();
  }
}
