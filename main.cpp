#include <iostream>

#include "Eigen/Eigen"
#include "Eigen/Geometry"

int main() {
  // Eigen::Quaterniond angle_axis(1, 1, 2, 0);
  // angle_axis.normalize();
  // auto rotMat = angle_axis.toRotationMatrix();
  // Eigen::AngleAxisd angle_axisd;
  // angle_axisd.fromRotationMatrix(rotMat);
  // std::cout << angle_axisd.axis() << std::endl;
  // Eigen::Quaterniond value(0, 1, 0, 0);
  // std::cout << value.toRotationMatrix() << std::endl;

  // double angle_x = -0.012;
  // Eigen::Quaterniond q_x(cos(angle_x/2.0),sin(angle_x/2.0),0,0);
  // std::cout <<  Eigen::AngleAxisd().fromRotationMatrix(q_x.toRotationMatrix()).axis() << std::endl;
  
  // Eigen::AngleAxisd roll_tmp(-1.580, Eigen::Vector3d::UnitX());
  // Eigen::AngleAxisd pitch_tmp(0.012, Eigen::Vector3d::UnitY());
  // Eigen::AngleAxisd yaw_tmp(-1.55, Eigen::Vector3d::UnitZ());
  // Eigen::Quaterniond delta_q = yaw_tmp * pitch_tmp * roll_tmp;
  // std::cout << delta_q.toRotationMatrix() << std::endl;
  // std::cout << yaw_tmp.toRotationMatrix() << std::endl;

  // Eigen::Quaterniond delta2_q = pitch_tmp * roll_tmp;
  // std::cout << delta2_q.toRotationMatrix() << std::endl;

  Eigen::VectorXd vec_data(3);
  vec_data << 10, 20, 30;
  std::cout << vec_data.maxCoeff() << std::endl;
  std::cout << vec_data.minCoeff() << std::endl;
}