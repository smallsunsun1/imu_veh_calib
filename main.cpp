#include <iostream>
#include "Eigen/Geometry"
#include "Eigen/Eigen"

int main() {
    Eigen::Quaterniond angle_axis(1, 1, 2, 0);
    angle_axis.normalize();
    auto rotMat = angle_axis.toRotationMatrix();
    Eigen::AngleAxisd angle_axisd;
    angle_axisd.fromRotationMatrix(rotMat);
    std::cout << angle_axisd.axis() << std::endl;
}