#include "data_selection.h"
#include "utils.h"

#define PI 3.1415926

namespace data_selection {

void SelectData(OdomDataList &odo_datas, CamDataList &cam_datas, std::vector<data_selection::SyncData> &sync_result) {
  StartPosAlign(odo_datas, cam_datas);
  // get rid of cam data ( deltaTheta = nan, tlc_length<1e-4,axis(1)<0.96), and align cam and odo data
  CamOdoAlign(odo_datas, cam_datas, sync_result);

  // get rid of the odo data whose distance is less than 1e-4 and whose theta sign isn't same
  std::vector<data_selection::SyncData> vec_sync_tmp;
  OdomDataList odo_matches;
  CamDataList cam_matches;
  int size = std::min(cam_datas.size(), odo_datas.size());

  int nFlagDiff = 0;
  for (int i = 0; i < size; ++i) {
    if (fabs(odo_datas[i].v_left) < 1e-3 || fabs(odo_datas[i].v_right) < 1e-3) continue;
    // the sign is opposite
    if ((odo_datas[i].v_right - odo_datas[i].v_left) * cam_datas[i].delta_theta < 0.0)  // rL == rR
    {
      nFlagDiff++;
      continue;
    }
    odo_matches.push_back(odo_datas[i]);
    cam_matches.push_back(cam_datas[i]);
    vec_sync_tmp.push_back(sync_result[i]);
  }
  std::cout << "nFlagDiff = " << nFlagDiff << std::endl;
  sync_result.swap(vec_sync_tmp);
  cam_datas.swap(cam_matches);
  odo_datas.swap(odo_matches);
}

void CamOdoAlign(OdomDataList &odo_datas, CamDataList &cam_datas, SyncDataList &sync_result) {
  int id_odo = 2;
  OdomDataList odo_datas_tmp;
  // CamDataList cam_datas_tmp;

  for (int i = 3; unsigned(i) < cam_datas.size() - 3; ++i) {
    if (std::isnan(cam_datas[i].delta_theta)) {
      DEBUG_LOG();
      continue;
    }

    double tlc_length = cam_datas[i].tlc.norm();
    if (tlc_length < 1e-4) {
      DEBUG_LOG();
      continue;
    }
    
    if (cam_datas[i].axis(1) > -0.96) {
      std::cout << cam_datas[i].axis(1) << std::endl;
      continue;
    }

    OdomData odo_start, odo_end;
    id_odo -= 2;
    double t_interval = fabs(odo_datas[id_odo].time - cam_datas[i].start_t);
    while (fabs(odo_datas[++id_odo].time - cam_datas[i].start_t) < t_interval)
      t_interval = fabs(odo_datas[id_odo].time - cam_datas[i].start_t);
    int odo_start_id = id_odo - 1;
    odo_start = odo_datas[odo_start_id];

    id_odo -= 2;
    t_interval = fabs(odo_datas[id_odo].time - cam_datas[i].end_t);
    while (fabs(odo_datas[++id_odo].time - cam_datas[i].end_t) < t_interval)
      t_interval = fabs(odo_datas[id_odo].time - cam_datas[i].end_t);
    int odo_end_id = id_odo - 1;
    odo_end = odo_datas[odo_end_id];

    if (odo_end_id - odo_start_id == 0) {
      DEBUG_LOG();
      continue;
    }

    // odo data: insert value
    double v_left_start, v_left_end;
    double v_right_start, v_right_end;

    if (cam_datas[i].start_t > odo_start.time) {
      float alpha = (cam_datas[i].start_t - odo_datas[odo_start_id].time) /
                    (odo_datas[odo_start_id + 1].time - odo_datas[odo_start_id].time);
      alpha = fabs(alpha);
      v_left_start = (1 - alpha) * odo_datas[odo_start_id].v_left + alpha * odo_datas[odo_start_id + 1].v_left;
      v_right_start = (1 - alpha) * odo_datas[odo_start_id].v_right + alpha * odo_datas[odo_start_id + 1].v_right;
    } else if (cam_datas[i].start_t < odo_start.time) {
      float alpha = (odo_datas[odo_start_id].time - cam_datas[i].start_t) /
                    (odo_datas[odo_start_id].time - odo_datas[odo_start_id - 1].time);
      alpha = fabs(alpha);
      v_left_start = (1 - alpha) * odo_datas[odo_start_id].v_left + alpha * odo_datas[odo_start_id - 1].v_left;
      v_right_start = (1 - alpha) * odo_datas[odo_start_id].v_right + alpha * odo_datas[odo_start_id - 1].v_right;
    } else {
      v_left_start = odo_start.v_left;
      v_right_start = odo_start.v_right;
    }
    odo_datas[odo_start_id].time = cam_datas[i].start_t;
    odo_datas[odo_start_id].v_left = v_left_start;
    odo_datas[odo_start_id].v_right = v_right_start;

    if (cam_datas[i].end_t > odo_end.time) {
      float alpha =
          (cam_datas[i].end_t - odo_datas[odo_end_id].time) / (odo_datas[odo_end_id + 1].time - odo_datas[odo_end_id].time);
      alpha = fabs(alpha);
      v_left_end = (1 - alpha) * odo_datas[odo_end_id].v_left + alpha * odo_datas[odo_end_id + 1].v_left;
      v_right_end = (1 - alpha) * odo_datas[odo_end_id].v_right + alpha * odo_datas[odo_end_id + 1].v_right;
    } else if (cam_datas[i].end_t < odo_end.time) {
      float alpha =
          (odo_datas[odo_end_id].time - cam_datas[i].end_t) / (odo_datas[odo_end_id].time - odo_datas[odo_end_id - 1].time);
      alpha = fabs(alpha);
      v_left_end = (1 - alpha) * odo_datas[odo_end_id].v_left + alpha * odo_datas[odo_end_id - 1].v_left;
      v_right_end = (1 - alpha) * odo_datas[odo_end_id].v_right + alpha * odo_datas[odo_end_id - 1].v_right;
    } else {
      v_left_end = odo_end.v_left;
      v_right_end = odo_end.v_right;
    }
    odo_datas[odo_end_id].time = cam_datas[i].end_t;
    odo_datas[odo_end_id].v_left = v_left_end;
    odo_datas[odo_end_id].v_right = v_right_end;

    // get the average ang_vel and lin_vel between camDatas[i].start_t and camDatas[i].end_t
    data_selection::OdomData odo_tmp;            // odo_tmp
    odo_tmp.time = odo_datas[odo_start_id].time;  // odo_tmp
    data_selection::SyncData sync_tmp;
    if (odo_end_id - odo_start_id > 1) {
      double dis_left_sum = 0.0, dis_right_sum = 0.0;
      for (int j = odo_start_id; j < odo_end_id; j++) {
        dis_left_sum += (odo_datas[j + 1].time - odo_datas[j].time) * (odo_datas[j + 1].v_left + odo_datas[j].v_left) / 2.0;
        dis_right_sum +=
            (odo_datas[j + 1].time - odo_datas[j].time) * (odo_datas[j + 1].v_right + odo_datas[j].v_right) / 2.0;
      }
      double T = cam_datas[i].end_t - cam_datas[i].start_t;
      odo_tmp.v_left = dis_left_sum / T;
      odo_tmp.v_right = dis_right_sum / T;

      sync_tmp.T = T;
      sync_tmp.velocity_left = dis_left_sum / T;
      sync_tmp.velocity_right = dis_right_sum / T;

      // 1: robot      camera    robot      camera
      //         x               z        |            x              x
      //        y               -x      |            y              z
      //       z               y         |            z               y

      sync_tmp.scan_match_results[0] = cam_datas[i].tlc[0];
      sync_tmp.scan_match_results[1] = cam_datas[i].tlc[2];
      sync_tmp.scan_match_results[2] = cam_datas[i].delta_theta;

      sync_tmp.tcl_cam = -cam_datas[i].Rcl * cam_datas[i].tlc;
      sync_tmp.qcl_cam = Eigen::Quaterniond(cam_datas[i].Rcl);
      sync_tmp.angle = -cam_datas[i].delta_theta;  // should be cl
      sync_tmp.axis = cam_datas[i].axis;
      sync_tmp.start_time = cam_datas[i].start_t;

    } else if (odo_end_id - odo_start_id == 1) {
      double T = cam_datas[i].end_t - cam_datas[i].start_t;
      double ave_v_left = (v_left_start + v_left_end) / 2.0;
      double ave_v_right = (v_right_start + v_right_end) / 2.0;

      odo_tmp.v_left = ave_v_left;
      odo_tmp.v_right = ave_v_right;

      sync_tmp.T = T;
      sync_tmp.velocity_left = ave_v_left;
      sync_tmp.velocity_right = ave_v_right;

      // 1: robot      camera    robot      camera
      //         x               z        |            x              x
      //        y               -x      |            y              z
      //       z               y         |            z               y

      sync_tmp.scan_match_results[0] = cam_datas[i].tlc[0];
      sync_tmp.scan_match_results[1] = cam_datas[i].tlc[2];
      // double angle_tmp = (camDatas[i].theta_y > 0.0?1:(-1)) * camDatas[i].deltaTheta;
      sync_tmp.scan_match_results[2] = cam_datas[i].delta_theta;

      sync_tmp.tcl_cam = -cam_datas[i].Rcl * cam_datas[i].tlc;
      sync_tmp.qcl_cam = Eigen::Quaterniond(cam_datas[i].Rcl);
      sync_tmp.angle = -cam_datas[i].delta_theta;  // should be cl
      sync_tmp.axis = cam_datas[i].axis;
      sync_tmp.start_time = cam_datas[i].start_t;
    }

    // cam_datas_tmp.push_back(cam_datas[i]);
    odo_datas_tmp.push_back(odo_tmp);  // odo_tmp
    sync_result.push_back(sync_tmp);

    id_odo--;
  }
  // cam_datas.swap(cam_datas_tmp);
  odo_datas.swap(odo_datas_tmp);
}

void StartPosAlign(OdomDataList &odo_datas, CamDataList &cam_datas) {
  double t0_cam = cam_datas[0].start_t;
  double t0_odo = odo_datas[0].time;
  if (t0_cam >= t0_odo) {
    double delta_t = fabs(odo_datas[0].time - t0_cam);

    int odo_start = 0;
    while (fabs(odo_datas[++odo_start].time - t0_cam) < delta_t) delta_t = fabs(odo_datas[odo_start].time - t0_cam);
    int odo_aligned = odo_start - 1;
    odo_datas.erase(odo_datas.begin(), odo_datas.begin() + odo_aligned);
    // for (int i = 0; i < odo_aligned; ++i) odo_datas.erase(odo_datas.begin());

    std::cout << std::fixed << "aligned start position1: " << cam_datas[0].start_t << "  " << odo_datas[0].time
              << std::endl;
  } else if (t0_odo > t0_cam) {
    double delta_t = fabs(cam_datas[0].start_t - t0_odo);

    int cam_start = 0;
    while (fabs(cam_datas[++cam_start].start_t - t0_odo) < delta_t) delta_t = fabs(cam_datas[cam_start].start_t - t0_odo);
    int cam_aligned = cam_start - 1;
    for (int i = 0; i < cam_aligned; ++i) cam_datas.erase(cam_datas.begin());

    std::cout << std::fixed << "aligned start position2 : " << cam_datas[0].start_t << "  " << odo_datas[0].time
              << std::endl;
  }
}
}  // namespace data_selection