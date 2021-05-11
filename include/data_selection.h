#ifndef INCLUDE_DATA_SELECTION_
#define INCLUDE_DATA_SELECTION_

#include <Eigen/Dense>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

class data_selection {
 public:
  struct odo_data {
    double time;     // 对应时间
    double v_left;   // 左轮速度
    double v_right;  // 右轮速度
  };
  struct cam_data {
    double start_t;        // 相机数据开始时间
    double end_t;          // 相机数据结束时间
    double deltaTheta;     // 旋转角度
    Eigen::Vector3d axis;  // 旋转轴
    Eigen::Matrix3d Rcl;   // 旋转矩阵
    Eigen::Vector3d tlc;   // 平移向量
  };

  // 两两匹配的同步数据结构体，用于存储同步后的Odom和Cam数据
  struct sync_data {
    // 阶段
    double T;
    // 左右轮速度
    double velocity_left;
    double velocity_right;
    // double velocity;
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
    double startTime;
  };
  void startPosAlign(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas);
  void selectData(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,
                  std::vector<data_selection::sync_data> &sync_result);

  void camOdoAlign(std::vector<odo_data> &odoDatas, std::vector<cam_data> &camDatas,
                   std::vector<sync_data> &sync_result);
  using OdomDataList =  std::vector<data_selection::odo_data>;
  using CamDataList = std::vector<data_selection::cam_data>;
  using SyncDataList = std::vector<data_selection::sync_data>;
  data_selection();
};

#endif /* INCLUDE_DATA_SELECTION_ */
