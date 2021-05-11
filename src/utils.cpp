#include <chrono>

#include "utils.h"

std::string colouredString(std::string str, std::string colour, std::string option)
{
  // double time_now = ros::Time::now().toSec();
  double time_now = std::chrono::high_resolution_clock::now().time_since_epoch().count();
  std::string time_string = std::to_string(time_now);
  return "[" + time_string + "]: " + option + colour + str + RESET;
}
