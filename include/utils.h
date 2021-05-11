#ifndef INCLUDE_UTILS_
#define INCLUDE_UTILS_

#include <string>

#include "data_selection.h"

const std::string RESET = "\033[0m";
const std::string BLACK = "0m";
const std::string RED = "1m";
const std::string GREEN = "2m";
const std::string YELLOW = "3m";
const std::string BLUE = "4m";
const std::string WHITE = "7m";
const std::string BOLD = "\033[1;3";
const std::string REGULAR = "\033[0;3";
const std::string UNDERLINE = "\033[4;3";
const std::string BACKGROUND = "\033[4";

/*!
 * \brief colouredString. Print coloured string
 * in terminal.
 * \param str. Input string.
 * \param colour. Colour option: BLACK, RED,
 * GREEN, YELLOW, BLUE, WHITE.
 * \param option. Char type option: BOLD, REGULAR,
 * UNDERLINE.
 * \return
 */
std::string colouredString(std::string str, std::string colour, std::string option);

/*
 * 用于字符串切分的函数
 */
std::vector<std::string> StrSplit(const std::string& str_data, const std::string& seperator);

/*
 * 使用该函数的必备条件是输入文件内的数据格式为：
 * time,v_left,v_right
 */
data_selection::OdomDataList LoadOdomData(const std::string& filename);

/*
 * 使用该函数的必备条件是输入文件内的数据格式为：
 * start_time, end_time, delta_theta, angle_axis_x, angle_axis_y, angle_axis_z
 */
data_selection::CamDataList LoadCamData(const std::string& filename);

#endif /* INCLUDE_UTILS_ */
