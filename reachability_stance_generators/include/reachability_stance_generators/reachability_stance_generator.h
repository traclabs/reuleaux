/********************************************************************************
Copyright (c) 2021, TRACLabs, Inc.
All rights reserved.
********************************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>
#include <stance_generator_base/stance_generator_base.h>
#include <craftsman_msgs/GenerateStance.h>
#include <geometry_msgs/PoseStamped.h>

#include <reachability_msgs/WorkSpace.h>
#include <reachability/sphere_discretization.h>
#include <reachability/helpers.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace reachability_stance_generators
{

class ReachabilityStanceGenerator : public stance_generator_base::StanceGeneratorBase
{
public:
  ReachabilityStanceGenerator();
  ~ReachabilityStanceGenerator();
  bool generateStance(const craftsman_msgs::GenerateStance::Request &req,
		      std::vector<geometry_msgs::PoseStamped> &stances);

  void associatePose(std::vector<std::pair<geometry_msgs::Point, std::vector<geometry_msgs::Pose> > >& basePoses,
		     const std::vector<geometry_msgs::PoseStamped> & grasp_poses,
		     const reachability_msgs::WorkSpace& winv);

 protected:
  reachability_msgs::WorkSpace winv_;
  bool inv_init_;  
};
}

