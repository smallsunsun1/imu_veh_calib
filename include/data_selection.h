#ifndef INCLUDE_DATA_SELECTION_
#define INCLUDE_DATA_SELECTION_

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace data_selection {
struct OdomData {
  double time;     // 对应时间
  double v_left;   // 左轮角速度
  double v_right;  // 右轮角速度
};
struct CamData {
  double start_t;        // 相机数据开始时间
  double end_t;          // 相机数据结束时间
  double delta_theta;    // 旋转角度
  Eigen::Vector3d axis;  // 旋转轴
  Eigen::Matrix3d Rcl;   // 旋转矩阵
  Eigen::Vector3d tlc;   // 平移向量
};

// 两两匹配的同步数据结构体，用于存储同步后的Odom和Cam数据
struct SyncData {
  // 表示当前数据的时间间隔
  double T;
  // 左右轮速度
  double velocity_left;
  double velocity_right;
  // camera data : x y yaw , x  y from tlc (not tcl)
  double scan_match_results[3];  // correct lx ly by R_x
  // Estimated rototranslation based on odometry params.
  double o[3];
  // Estimated disagreement  sm - est_sm
  double est_sm[3];
  double err_sm[3];  //  s  - (-) l (+) o (+) l
  // Other way to estimate disagreement:   l (+) s  - o (+) l
  double err[3];
  int mark_as_outlier;
  // tcl_cam and qcl_cam are original data(not correted by R_x)
  Eigen::Vector3d tcl_cam;  // 06/06
  Eigen::Quaterniond qcl_cam;
  // 角度
  double angle;
  // 轴
  Eigen::Vector3d axis;
  double start_time;
};
using OdomDataList = std::vector<OdomData>;
using CamDataList = std::vector<CamData>;
using SyncDataList = std::vector<SyncData>;
void StartPosAlign(OdomDataList &odo_datas, CamDataList &cam_datas);
void SelectData(OdomDataList &odo_datas, CamDataList &cam_datas, SyncDataList &sync_result);

void CamOdoAlign(OdomDataList &odo_datas, CamDataList &cam_datas, SyncDataList &sync_result);
};

#endif /* INCLUDE_DATA_SELECTION_ */
