#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <dynamixel_workbench_msgs/Javis_msgs.h>
#include <dynamixel_workbench_msgs/javis_des.h>

#include "javis_pkg_arm/arm_controller.h"

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 6> J_Mat;
typedef Eigen::Matrix<double, 6, 3> J_M;

using namespace Eigen;
using namespace std;

Vector3d _Pdes_;

void desCallback(const dynamixel_workbench_msgs::javis_des& msg)
{
  cout << "OK" << endl;
  for(int i = 0 ; i < 3 ; i++){
    _Pdes_(i,0) = msg.des[i];
  }
}

int main(int argc, char **argv)
{
  Vector6d Pdes, Perr, Pcur, Perr_pre, Perr_dif;
  Vector6d _q, _dq;
  Vector4d origin, ee;
  Matrix6d mat, jacobian;
  Matrix4d fk;
  J_M mat_j;
  J_Mat jacobian_;
  Vector3d _Pdes, _Perr, _Pcur, _Perr_pre, _Perr_dif;
  Vector3d init(0.36, 0.0, 0.36);
  double dt = 0.05;
  double Kp = 1.0 / dt, K_sibal = 4;
  double lamda = 0.03;
  double lamda2 = lamda*lamda;
//  double Kp_x = 0.05 / dt;
//  double Kp_y = 1.0 / dt;
//  double Kp_z = 1.0 / dt;

  Pdes.setZero();
  Perr.setZero();
  Pcur.setZero();

  _Pdes.setZero();
  _Perr.setZero();
  _Pcur.setZero();

  _q.setZero();
  _dq.setZero();
  mat.setZero();
  mat_j.setZero();
  jacobian.setZero();
  fk.setZero();
  ee.setZero();
  jacobian_.setZero();

  origin << 0, 0, 0, 1;

  ros::init(argc, argv, "arm_controller");
  ros::NodeHandle nh;

  dynamixel_workbench_msgs::Javis_msgs msg;
  ros::Publisher mani_pub = nh.advertise<dynamixel_workbench_msgs::Javis_msgs>("manipulator", 1000);
  ros::Subscriber sub = nh.subscribe("destination", 1000, desCallback);
  javis_arm arm;

  _Pdes_ = init;
  _q << 0.0, PI/2, -PI/2, 0.0, 0.0, 0.0;
  ros::Rate rate(20);

  while(ros::ok()){
    _Pdes = _Pdes_;

    cout << _Pdes << endl;
    fk = arm.forward_kinematics(_q);
    cout << "\n fk : \n" << fk<< endl;

    ee = fk * origin;

    _Pcur = ee.block(0, 0, 3, 1);
    cout << "\n Pcur : \n" << _Pcur << endl;

    _Perr = - _Pdes + _Pcur;
    cout << "\n Perr : \n" << _Perr << endl;

    //Pseudo Inverse
    jacobian_ = arm.jacobian_(_q);
    cout << "\n J : \n" << jacobian_ << endl;

    mat_j = jacobian_.transpose() * ((jacobian_ * jacobian_.transpose() + ((lamda2 + _Perr.norm()*K_sibal)* arm.I3)).inverse());
    cout << "\n J_psuedo : \n" << mat_j << endl;

    _dq = mat_j * _Perr * Kp;
    cout << "\n _dq : \n" << _dq << endl;

    _q += _dq * dt;

    for(int i = 0 ; i < 6 ; i++){
      msg.angle[i] = _q(i,0);
    }

    cout << "\n _q : \n" << _q << endl;
    cout << "#####################################" << endl;
    mani_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }
}
