/*******************************************************
 * Copyright (C) 2019, SLAM Group, Megvii-R
 *******************************************************/

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <queue>
#include <thread>

#include "data_selection.h"
#include "solveQyx.h"
#include "utils.h"

using namespace data_selection;

// extract images with same timestamp from two topics
void CalcProcess(data_selection::OdomDataList& odom_datas, data_selection::CamDataList& cam_datas) {
  SolveQyx cSolveQyx;
  std::cout << "============ calibrating... ===============" << std::endl;
  std::vector<data_selection::SyncData> sync_result;
  SelectData(odom_datas, cam_datas, sync_result);

  std::cout << "sync_result size: " << sync_result.size() << std::endl;

  // first estimate the Ryx and correct tlc of camera
  Eigen::Matrix3d Ryx;
  cSolveQyx.EstimateRyx(sync_result, Ryx);
  std::cout << "Ryx Matrix\n" << Ryx << std::endl;
  cSolveQyx.CorrectCamera(sync_result, cam_datas, Ryx);

  // calibrate r_L  r_R  axle  lx  ly  yaw
  cSolver cSolve;
  cSolver::calib_result paras;          // radius_l,radius_r,axle,l[3]
  cSolve.calib(sync_result, 4, paras);  // by svd

  // secondly estimate the Ryx
  cSolveQyx.EstimateRyx(sync_result, Ryx);

  // refine all the extrinal parameters
  cSolveQyx.RefineExPara(sync_result, paras, Ryx);
}

int main(int argc, char* argv[]) {
  std::string odom_data_file = argv[1];
  std::string camera_data_file = argv[2];
  data_selection::OdomDataList odom_datas = LoadOdomData(odom_data_file);
  data_selection::CamDataList cam_datas = LoadCamData(camera_data_file);

  CalcProcess(odom_datas, cam_datas);

  return 0;
}
