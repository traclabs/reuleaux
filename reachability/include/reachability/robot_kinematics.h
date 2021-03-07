#pragma once

#include <ros/ros.h>
#include <trac_ik/trac_ik.hpp>

/**
 * @class RobotKinematics
 */
class RobotKinematics
{
 public:
  RobotKinematics(ros::NodeHandle _pnh);
  bool init();
  bool setupChain(KDL::Chain &_chain,
		  KDL::JntArray &_qmin,
		  KDL::JntArray &_qmax,
		  KDL::JntArray &_qnom);
  bool isIKSuccess(const std::vector<double> &_pose,
		   std::vector<double> &_joints,
		   int &_solns);
  
 protected:
  ros::NodeHandle pnh_;

  std::string chain_start_;
  std::string chain_end_;
  std::string urdf_param_;
  double eps_;
  double timeout_;

  KDL::Chain chain_;
  KDL::JntArray qmin_;
  KDL::JntArray qmax_;
  KDL::JntArray qnom_;

  std::shared_ptr<TRAC_IK::TRAC_IK_SOLVER> solver_;
};
