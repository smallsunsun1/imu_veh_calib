/*******************************************************
 * Copyright (C) 2019, SLAM Group, Megvii-R
 *******************************************************/

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>

#include "data_selection.h"
#include "solveQyx.h"

// record the first frame calculated successfully
bool fisrt_frame = true;
Eigen::Matrix3d Rwc0;
Eigen::Vector3d twc0;
// decide if the frequent is decreased
bool halfFreq = false;
int frame_index = 0;

// extract images with same timestamp from two topics
void CalcProcess(DataSelection::OdomDataList& odom_datas, DataSelection::CamDataList& cam_datas) {
  SolveQyx cSolveQyx;
  std::cout << "============ calibrating... ===============" << std::endl;
  DataSelection ds;
  std::vector<DataSelection::sync_data> sync_result;
  ds.selectData(odom_datas, cam_datas, sync_result);

  // first estimate the Ryx and correct tlc of camera
  Eigen::Matrix3d Ryx;
  cSolveQyx.estimateRyx(sync_result, Ryx);
  cSolveQyx.correctCamera(sync_result, cam_datas, Ryx);

  // calibrate r_L  r_R  axle  lx  ly  yaw
  cSolver cSolve;
  cSolver::calib_result paras;          // radius_l,radius_r,axle,l[3]
  cSolve.calib(sync_result, 4, paras);  // by svd

  // secondly estimate the Ryx
  cSolveQyx.estimateRyx(sync_result, Ryx);

  // refine all the extrinal parameters
  cSolveQyx.refineExPara(sync_result, paras, Ryx);
}

int main(int argc, char** argv) {
  std::string config_file = argv[1];
  std::cout << "config file: " << config_file << std::endl;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  std::string WHEEL_TOPIC, IMAGE_TOPIC;
  fsSettings["wheel_topic"] >> WHEEL_TOPIC;
  fsSettings["image_topic"] >> IMAGE_TOPIC;

  std::cout << "IMAGE TOPIC: " << IMAGE_TOPIC << " : "
            << "WHEEL TOPIC: " << WHEEL_TOPIC << std::endl;

  return 0;
}
