
#include <reachability/helpers.h>


/**
 * @file readReachMapToRosMessage
 */
bool readReachMapToRosMessage(std::string _reachability_fullname,
			      std::string _reference_frame,
			      reachability_msgs::WorkSpace &_ws)
{
  hdf5_dataset::Hdf5Dataset h5(_reachability_fullname);
  if(!h5.open())
  {
    ROS_ERROR("Error opening H5 file %s", _reachability_fullname.c_str());
    return false;
  }

  MultiMapPtr pose_col_filter;
  MapVecDoublePtr sphere_col;
  float res;
  h5.loadMapsFromDataset(pose_col_filter, sphere_col, res);

  // Creating messages
  _ws.header.stamp = ros::Time::now();
  _ws.header.frame_id = _reference_frame;
  _ws.resolution = res;

  for (MapVecDoublePtr::iterator it = sphere_col.begin();
       it != sphere_col.end();
       ++it)
  {
    // Fill sphere information
    reachability_msgs::WsSphere wss;
    wss.point.x = (*it->first)[0];
    wss.point.y = (*it->first)[1];
    wss.point.z = (*it->first)[2];
    wss.ri = it->second;
    
    // Fill individual points per sphere information
    for (MultiMapPtr::iterator it1 = pose_col_filter.lower_bound(it->first);
	 it1 != pose_col_filter.upper_bound(it->first);
	 ++it1)
    {
      geometry_msgs::Pose pp;
      pp.position.x = it1->second->at(0);
      pp.position.y = it1->second->at(1);
      pp.position.z = it1->second->at(2);
      pp.orientation.x = it1->second->at(3);
      pp.orientation.y = it1->second->at(4);
      pp.orientation.z = it1->second->at(5);
      pp.orientation.w = it1->second->at(6);
      wss.poses.push_back(pp);
    }
    // Add full data for one sphere
    _ws.WsSpheres.push_back(wss);
  }

  return true;
}

bool saveReachabilityMapMessage(std::string path,
				reachability_msgs::WorkSpace ws)
{
  std::ofstream f((path + ".tmp").c_str());

  std::string save_string_ = "mv " + path + ".tmp " + path;
  std::mutex reach_lock;
  
  if(f.good())
  {
    try
    {
      cereal::BinaryOutputArchive oarch(f);
      {
	std::lock_guard<std::mutex> lock(reach_lock);
	oarch(ws);
      }
      f.flush();
      f.close();
      
      int rc = system(save_string_.c_str());
      if(rc == 0)
      {
	std::lock_guard<std::mutex> lock(reach_lock);
      }
    } // end try
    catch(...)
    {
      ROS_ERROR("Could not save to %s", path.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("f is not good...");
    return false;
  }

  f.close();
  return true;
}

/**
 * @function laodReachabilityMapMessage
 */
bool loadReachabilityMapMessage(std::string path,
				reachability_msgs::WorkSpace &ws)
{
  std::ifstream f(path.c_str());
  std::mutex reach_lock;

  if (f.good())
  {
    try
    {
      reach_lock.lock();
      cereal::BinaryInputArchive iarch(f);
      iarch(ws);
      reach_lock.unlock();
    }
    catch (...)
    {
      ROS_ERROR("Error in reading binary file: %s",
		path.c_str());
      reach_lock.unlock();
      return false;
    }
  }
  else
  {
    ROS_ERROR("loadReachabilityMapMessage: f is NOT good");
    return false;
  }
  ROS_WARN("Things worked fine.");
  f.close();
  return true;
}
