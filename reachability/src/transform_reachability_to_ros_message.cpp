/**
 * @file transform_reachability_to_ros_message.cpp
 */
#include <ros/ros.h>
#include <ros/package.h>
#include <reachability/helpers.h>
#include <mutex>
#include <fstream>

/*******************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transform_reachability_to_ros_message");
  ros::NodeHandle pnh_("~");
  ros::NodeHandle nh_;

  std::string reachability_map_;
  std::string reachability_fullname_;
  std::string reference_frame_;
  std::string reachability_message_ = "reach_map_test";

  std::string path(ros::package::getPath("reachability") + "/maps/");
  
  pnh_.getParam("reachability_map", reachability_map_);
  pnh_.getParam("reachability_message", reachability_message_);
  pnh_.getParam("reference_frame", reference_frame_);

  reachability_fullname_ = path + reachability_map_ + std::string(".h5");

  reachability_msgs::WorkSpace ws;
  if(!readReachMapToRosMessage(reachability_fullname_,
			       reference_frame_,
			       ws))
    return 2;

  // Store in binary
  std::string map_path = path + reachability_message_;

  saveReachabilityMapMessage(map_path, ws);
  return 0;
}

