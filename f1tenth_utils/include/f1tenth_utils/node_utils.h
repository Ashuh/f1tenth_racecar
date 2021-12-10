#ifndef F1TENTH_UTILS_NODE_UTILS_H
#define F1TENTH_UTILS_NODE_UTILS_H

#include <string>

#include <ros/ros.h>

template <typename T>
bool getClosestParam(const ros::NodeHandle& nh, const std::string& key, T& result)
{
  std::string key_found;

  if (nh.searchParam(key, key_found))
  {
    nh.getParam(key_found, result);
  }
  else
  {
    return false;
  }
}

#endif  // F1TENTH_UTILS_NODE_UTILS_H
