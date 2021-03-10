/**
 * @file helpers.h
 */
#pragma once

#include <reachability/hdf5_dataset.h>
#include <reachability_msgs/WorkSpace.h>
#include <mutex>

#include <cache_utils/trac_message_serialization.h>

// ******************* reachability_msgs::WorkSpace ********************
namespace cereal
{
template<class Archive>
void save(Archive &ar,
          const reachability_msgs::WorkSpace &ws,
          const unsigned int version)
{
  copy_buffer<reachability_msgs::WorkSpace, Archive>(ws, ar);
}

template<class Archive>
void load(Archive &ar,
          reachability_msgs::WorkSpace &jt,
          const unsigned int version)
{
  jt = load_buffer<reachability_msgs::WorkSpace, Archive>(ar);
}

} // namespace cereal


bool readReachMapToRosMessage(std::string _reachability_fullname,
			      std::string _reference_frame,
			      reachability_msgs::WorkSpace &_ws);

bool saveReachabilityMapMessage(std::string path,
				reachability_msgs::WorkSpace ws);

bool loadReachabilityMapMessage(std::string path,
				reachability_msgs::WorkSpace &ws);
