#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg ){
   //TF odom => base_link
    static tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(odomsg->pose.pose.orientation.x,
                                     odomsg->pose.pose.orientation.y,
                                     odomsg->pose.pose.orientation.z,
                                     odomsg->pose.pose.orientation.w),
        tf::Vector3(odomsg->pose.pose.position.x/1000.0,
                    odomsg->pose.pose.position.y/1000.0,
                    odomsg->pose.pose.position.z/1000.0)),
        odomsg->header.stamp, "/robot_pose_ekf/odom_combined", "/base_footprint"));
      ROS_DEBUG("odometry frame sent");
      ROS_INFO("odometry frame sent");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pioneer_tf");
  ros::NodeHandle n;

  ros::Rate r(20);

  tf::TransformBroadcaster broadcaster;
  tf::Quaternion q;
  q.setRPY(3.141592,0,0);

  //subscribe to pose info
  ros::Subscriber pose_sub = n.subscribe<nav_msgs::Odometry>("RosAria/pose", 1, poseCallback);

  while(n.ok()){

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(), "/base_footprint","/base_link"));

    //base_link => laser
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q, tf::Vector3(0.25, 0.0, 0.0)),
        ros::Time::now(), "/base_link", "/laser"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q, tf::Vector3(0.25, 0.0, 0.0/*Must change later*/)),
        ros::Time::now(), "/base_link", "/camera_link"));

    ros::spinOnce();
    r.sleep();
  }
}
