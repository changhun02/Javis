#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

#define PI 3.141592

using namespace Eigen;
class javis_arm{

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 6> J_Mat;

public:
  double lamda, dt;
  Matrix6d I6;
  Matrix3d I3;
  Matrix4d End_e, Mat, Mat_res, T[6], T_res[6];

  javis_arm(){

    j.setZero();
    Pcur.setZero();
    Perr.setZero();
    _q.setZero();
    _dq.setZero();
    _th.setZero();
    Mat.setZero();
    origin.setZero();

    //Must change Lamda!!
    lamda = 0.03;
    dt = 0.05;
    Kp = 1.0;
    Mat_res.setIdentity();

    I6.setIdentity();
    I3.setIdentity();
    I6 = lamda*lamda*I6;
    //I3 = lamda*lamda*I3;
    end_effector.setZero();
    for(int i = 0 ; i < 6 ; i ++){
      T[i].setZero();
      T_res[i].setZero();
    }
  }

  ~javis_arm(){

  }


  Matrix4d forward_kinematics(Vector6d th){
    Mat_res.setIdentity();

    for(int i = 0; i < 6; i++){
      T[i] = transMatrix(th(i,0),alpha[i],a[i],d[i]);
      Mat_res  *= T[i];
      T_res[i]  = Mat_res;
      z_axis[i] = T[i].block(0, 2, 3, 1);
      pos[i]    = T[i].block(0, 3, 3, 1);
    }

    //Mat_res *= transMatrix(0,0,-0.075,0);
    Mat_res *= transMatrix(0,0,0.140,0);
    return Mat_res;
  }


  Matrix6d jacobian(Vector6d Q){ //dirldp wjdrlrngkr dlqfurdmfh sjgrl
    Vector3d Pos, P_3d;
    Matrix6d Jaco;
    Matrix4d Fk;
    Fk = forward_kinematics(Q);

    P_3d.setIdentity();
    P_3d = Fk.block(0,3,3,1);

    for(int i = 0 ; i < 6 ; i++){
      Pos = (P_3d - pos[i]).cross(z_axis[i]);

      Jaco(0,i) = Pos(0,0);
      Jaco(1,i) = Pos(1,0);
      Jaco(2,i) = Pos(2,0);

      Jaco(3,i) = z_axis[i](0,0);
      Jaco(4,i) = z_axis[i](1,0);
      Jaco(5,i) = z_axis[i](2,0);
    }

    return Jaco;
  }

  J_Mat jacobian_(Vector6d Q){
    Vector3d Pos, P_3d;
    J_Mat Jaco;
    Matrix4d Fk;
    Fk = forward_kinematics(Q);

    P_3d.setIdentity();
    P_3d = Fk.block(0,3,3,1);

    for(int i = 0 ; i < 6 ; i++){
      Pos = (P_3d - pos[i]).cross(z_axis[i]);

      Jaco(0,i) = Pos(0,0);
      Jaco(1,i) = Pos(1,0);
      Jaco(2,i) = Pos(2,0);
    }

    return Jaco;
  }
/*
  /*MatrixXd pose_callback(Vector3d pose_des, Vector3d pose_cur){
    Pcur = pose_cur;
    Pdes = pose_des;
    MatrixXd j = jacobian();
    Perr = Pcur - Pdes;
    _dq = (j + (lamda * lamda)*I6).inverse() * (Perr*Kp);
    _q += _dq;1000

    return _q;
  }*/

  /*Matrix4d translate(char axis, double dist){

    Matrix4d m;
    if(axis == 'x'){
      m.setIdentity();
      m(3, 1) = dist;
      return m;
    }

    else if(axis == 'z'){
      m.setIdentity();
      m(3, 2) = dist;
      return m;
    }
  }*/

  /*Matrix4d rotation(char axis, double th) {
    Matrix4d m;
    m.setIdentity();

    if(axis == 'x'){
      Eigen::Affine3d r =
          Eigen::Affine3d(Eigen::AngleAxisd(th, Eigen::Vector3d(1, 0, 0)));
      m.block(0,0,3,3) = r.rotation();
    }
    if(axis == 'y'){
      Eigen::Affine3d r =
          Eigen::Affine3d(Eigen::AngleAxisd(th, Eigen::Vector3d(0, 1, 0)));
      m.block(0,0,3,3) = r.rotation();
    }
    if(axis == 'z'){
      Eigen::Affine3d r =
          Eigen::Affine3d(Eigen::AngleAxisd(th, Eigen::Vector3d(0, 0, 1)));
      m.block(0,0,3,3) = r.rotation();
    }

    return m;
  }*/

  Matrix4d transMatrix(double th, double alpha, double a, double d){

    Matrix4d M;
    M.setZero();

    M(0,0) = cos(th);
    M(0,1) = -1.0*sin(th);
    M(0,2) = 0.0;
    M(0,3) = a;

    M(1,0) = sin(th)*cos(alpha);
    M(1,1) = cos(th)*cos(alpha);
    M(1,2) = -1.0*sin(alpha);
    M(1,3) = -1.0*sin(alpha)*d;

    M(2,0) = sin(th)*sin(alpha);
    M(2,1) = cos(th)*sin(alpha);
    M(2,2) = cos(alpha);
    M(2,3) = cos(alpha)*d;

    M(3,0) = 0.0;
    M(3,1) = 0.0;
    M(3,2) = 0.0;
    M(3,3) = 1.0;

    return M;
  }

  /*Matrix4d transMatrix(double th, double alpha, double a, double d){
    Matrix4d M;
    M.setZero();

    M = rotation('x',alpha)*translate(a,0,0)*rotation('z',th)*translate(0,0,d);
    return M;
  }*/

  Matrix4d translate(double x, double y, double z){
    Matrix4d A;
    A.setIdentity();
    Vector3d v(x,y,z);

    A.block(0,3,3,1) = v;
    return A;
  }
protected:

private:
  /*double alpha[6] = {0, PI/2, 0, 0, -PI/2, PI/2};
  double a[6]     = {0, 0, 0.250, 0.220, 0, 0};
  double d[6]     = {0.035, -0.053, 0, 0, 0.075, 0.075}; //d[0] = 0.035*/
/*
  double alpha[6] = {0, PI/2, 0, 0, PI/2, -PI/2};
  double a[6]     = {0, 0, -0.250, -0.220, 0, 0};
  double d[6]     = {0.035, 0, 0, 0.026, 0.075, 0.075};
*/
  double alpha[6] = {0, PI/2, 0, 0, PI/2, PI/2};
  double a[6]     = {0, 0, 0.250, 0.220, 0, 0};
  double d[6]     = {0.035, 0, 0, -0.026, 0.075, 0.075};
  Vector3d x_axis[6], y_axis[6], z_axis[6], pos[6];
  //Vector_axis y_axis[6], z_axis[6], pos[6];
  Vector3d Pcur, Perr;
  Vector3d end_effector, origin;
  Matrix6d j;
  Vector6d _dq, _q, _th;

  double Kp;
};

#endif // ARM_CONTROLLER_H
