#include "utils.h"

#include <chrono>
#include <fstream>
#include <regex>
#include <string>

#include "Eigen/Geometry"
#include "data_selection.h"

std::string colouredString(std::string str, std::string colour, std::string option) {
  double time_now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::string time_string = std::to_string(time_now);
  return "[" + time_string + "]: " + option + colour + str + RESET;
}

std::vector<std::string> StrSplit(const std::string& str_data, const std::string& seperator) {
  std::vector<std::string> result;
  std::regex re(seperator);
  std::sregex_token_iterator begin(str_data.begin(), str_data.end(), re, -1);
  std::sregex_token_iterator end;
  for (auto iter = begin; iter != end; ++iter) {
    result.push_back(iter->str());
  }
  return result;
}

data_selection::OdomDataList LoadOdomData(const std::string& filename) {
  std::ifstream in_f(filename, std::ios::in);
  std::string current_line;
  // 不需要第一行，strip
  std::getline(in_f, current_line);
  data_selection::OdomDataList results;
  while (std::getline(in_f, current_line)) {
    std::vector<std::string> splited_data = StrSplit(current_line, ",");
    data_selection::OdomData odom_temp;
    odom_temp.time = std::stod(splited_data[0]);
    odom_temp.v_left = std::stod(splited_data[1]);
    odom_temp.v_right = std::stod(splited_data[2]);
    results.push_back(std::move(odom_temp));
  }
  return results;
}

data_selection::CamDataList LoadCamData(const std::string& filename) {
  std::ifstream in_f(filename, std::ios::in);
  data_selection::CamDataList results;
  std::string current_line;
  // 不需要第一行，strip
  std::getline(in_f, current_line);
  while (std::getline(in_f, current_line)) {
    std::vector<std::string> splited_data = StrSplit(current_line, ",");
    data_selection::CamData cam_temp;
    cam_temp.start_t = std::stod(splited_data[0]);
    cam_temp.end_t = std::stod(splited_data[1]);
    cam_temp.delta_theta = std::stod(splited_data[2]);
    Eigen::Quaterniond angle_axis(std::stod(splited_data[3]), std::stod(splited_data[4]), std::stod(splited_data[5]),
                                  std::stod(splited_data[6]));
    angle_axis.normalize();
    cam_temp.tlc = {std::stod(splited_data[11]), std::stod(splited_data[12]), std::stod(splited_data[13])};
    Eigen::Quaterniond rot_value = {std::stod(splited_data[7]), std::stod(splited_data[8]), std::stod(splited_data[9]), std::stod(splited_data[10])};
    cam_temp.Rcl = rot_value.toRotationMatrix();
    Eigen::AngleAxisd rotate_axis2;
    rotate_axis2.fromRotationMatrix(cam_temp.Rcl);
    cam_temp.axis = rotate_axis2.axis();
    if (cam_temp.axis(1) > 0) {
      cam_temp.delta_theta *= -1;
      cam_temp.axis *= -1;
    }

    results.push_back(std::move(cam_temp));
  }
  return results;
}

