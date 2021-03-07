/**
 * @file robot_kinematics.cpp
 */
#include <reachability/robot_kinematics.h>

#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <ros/package.h>

/**
 * @function RobotKinematics
 */
RobotKinematics::RobotKinematics(ros::NodeHandle _pnh)
{
  pnh_ =  _pnh;  
}

/**
 * @function init
 */
bool RobotKinematics::init()
{
  pnh_.getParam("chain_start", chain_start_);
  pnh_.getParam("chain_end", chain_end_);
  pnh_.getParam("urdf_param", urdf_param_);

  eps_ = 1e-5;
  timeout_ = 0.005;

  if(chain_start_.size() == 0)
    return false;
  if(chain_end_.size() == 0)
    return false;
  if(urdf_param_.size() == 0)
    return false;


  if(!setupChain(chain_, qmin_, qmax_, qnom_))
    return false;
  
  solver_.reset(new TRAC_IK::TRAC_IK_SOLVER(chain_,
					    qmin_,
					    qmax_,
					    timeout_,
					    eps_,
					    TRAC_IK::SolveType::Speed));

  return true;
}

/**
 * @function setupChain
 */
bool RobotKinematics::setupChain(KDL::Chain &_chain,
				 KDL::JntArray &_qmin,
				 KDL::JntArray &_qmax,
				 KDL::JntArray &_qnom)
{
  urdf::Model robot_model;
  KDL::Tree tree;
  
  if (!robot_model.initParam(urdf_param_))
    return false;
  
  if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
  {      
    ROS_ERROR("Failed to extract kdl tree from xml robot description");
    return false;
  }

  if (!tree.getChain(chain_start_, chain_end_, _chain))
  {
    ROS_ERROR("Couldn't find chain %s to %s",
	      chain_start_.c_str(),
	      chain_end_.c_str());
    return false;
  }

  if (_chain.getNrOfJoints() == 0)
  {
    ROS_ERROR("Chain has 0 joints! Returning false");
    return false;
  }


  urdf::JointConstSharedPtr urdf_joint;
  std::vector<KDL::Segment> chain_segs = _chain.segments;
  std::vector<double> lower_limits, upper_limits;


  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {

    urdf_joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
    ROS_INFO_STREAM(chain_segs[i].getName() << " " << chain_segs[i].getJoint().getName());

    if (urdf_joint->type == urdf::Joint::UNKNOWN || urdf_joint->type == urdf::Joint::FIXED)
      continue;

    //if joint has limits, it's movable and we want it as part of our robot chain
    if (urdf_joint->type != urdf::Joint::CONTINUOUS) 
    {
      double lower, upper;
      if (urdf_joint->safety)
      {
	lower = std::max(urdf_joint->limits->lower, urdf_joint->safety->soft_lower_limit);
	upper = std::min(urdf_joint->limits->upper, urdf_joint->safety->soft_upper_limit);
      }
      else
      {
	lower = urdf_joint->limits->lower;
	upper = urdf_joint->limits->upper;
      }

      lower_limits.push_back(lower);
      upper_limits.push_back(upper);
    } // if urdf_joint
    else
    {
      lower_limits.push_back(-FLT_MAX);
      upper_limits.push_back(FLT_MAX);
    }
  } // int i = 0

  if(_chain.getNrOfJoints() != lower_limits.size())
  {
    ROS_ERROR("Chain joints (%lu) is different than lower limits size(%lu)",
	      _chain.getNrOfJoints(),
	      lower_limits.size());
    return false;
  }


  KDL::JntArray q(lower_limits.size());
  KDL::JntArray ll(lower_limits.size());
  KDL::JntArray ul(lower_limits.size());
  KDL::JntArray nom(lower_limits.size());
  
  for (uint j = 0; j < lower_limits.size(); j++)
  {
    ROS_INFO_STREAM("Joint " << j << " " << lower_limits[j] << " " << upper_limits[j]);
    nom(j) = (lower_limits[j] + upper_limits[j]) / 2.0;
    ll(j) = lower_limits[j];
    ul(j) = upper_limits[j];
  }

  _qmin = ll;
  _qmax = ul;
  _qnom = nom;
  
  return true;
}

/**
 * @function isIKSuccess
 */
bool RobotKinematics::isIKSuccess(const std::vector<double> &_pose,
				  std::vector<double> &_joints,
				  int &_solns)
{
  KDL::Frame ee_pose;
  ee_pose.p.x(_pose[0]);
  ee_pose.p.y(_pose[1]);
  ee_pose.p.z(_pose[2]);

  ee_pose.M = KDL::Rotation::Quaternion(_pose[3], _pose[4], _pose[5], _pose[6]);
  
  std::vector<KDL::JntArray> q_outs;
  if(solver_->CartToJnt(qnom_, ee_pose, q_outs) > 0)
  {
    _solns = q_outs.size();
    return true;
  }

  return false;
}
  
