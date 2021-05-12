#include <cmath>
#include <fstream>

#include "glog/logging.h"
#include "gflags/gflags.h"

#include "data_selection.h"
#include "solveQyx.h"
#include "utils.h"

DEFINE_string(odom_data, "/home/jiahesun/projects/imu_veh_calib/test_data/formatted_odometry.txt", "file where store the odom data");
DEFINE_string(cam_data, "/home/jiahesun/projects/imu_veh_calib/test_data/formatted_camera.txt", "file where store the imu data");

using namespace data_selection;

// extract images with same timestamp from two topics
void CalcProcess(data_selection::OdomDataList& odom_datas, data_selection::CamDataList& cam_datas) {
  SolveQyx cSolveQyx;
  LOG(INFO) << "============ calibrating... ===============";
  std::vector<data_selection::SyncData> sync_result;
  SelectData(odom_datas, cam_datas, sync_result);

  LOG(INFO) << "sync_result size: " << sync_result.size() << std::endl;

  // first estimate the Ryx and correct tlc of camera
  Eigen::Matrix3d Ryx;
  cSolveQyx.EstimateRyx(sync_result, Ryx);
  LOG(INFO) << "Ryx Matrix\n" << Ryx << std::endl;
  cSolveQyx.CorrectCamera(sync_result, cam_datas, Ryx);

  // calibrate r_L  r_R  axle  lx  ly  yaw
  CSolver cSolve;
  CSolver::CalibResult paras;          // radius_l,radius_r,axle,l[3]
  cSolve.Calib(sync_result, 4, paras);  // by svd

  // secondly estimate the Ryx
  cSolveQyx.EstimateRyx(sync_result, Ryx);

  // refine all the extrinal parameters
  cSolveQyx.RefineExPara(sync_result, paras, Ryx);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  std::string odom_data_file = FLAGS_odom_data;
  std::string camera_data_file = FLAGS_cam_data;
  // 加载Odom数据和Cam数据
  data_selection::OdomDataList odom_datas = LoadOdomData(odom_data_file);
  data_selection::CamDataList cam_datas = LoadCamData(camera_data_file);

  // 进行计算
  CalcProcess(odom_datas, cam_datas);

  return 0;
}
