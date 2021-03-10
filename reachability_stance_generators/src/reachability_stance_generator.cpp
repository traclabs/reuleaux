/********************************************************************************
Copyright (c) 2019, TRACLabs, Inc.
All rights reserved.
********************************************************************************/

#include <reachability_stance_generators/reachability_stance_generator.h>

using namespace reachability_stance_generators;

ReachabilityStanceGenerator::ReachabilityStanceGenerator()
{
  ROS_INFO_STREAM("ReachabilityStanceGenerator -- creating a naive stance generator!");

  //pnh.getParam("reference_frame", reference_frame_);
  std::string path(ros::package::getPath("reachability") + "/maps/");
  std::string inverse_reach_map_ = "inverse_fetch_5_ros_message";
  nh_.getParam("inverse_reach_map", inverse_reach_map_);
  
  std::string inverse_reach_map_fullname_;
  inverse_reach_map_fullname_ = path + inverse_reach_map_;
  
  inv_init_ = loadReachabilityMapMessage(inverse_reach_map_fullname_, winv_);
  if(!inv_init_)
    ROS_ERROR("Inv Reach not initialized correctly, it won't work. Fullname: %s",
	      inverse_reach_map_fullname_.c_str());
  
}

ReachabilityStanceGenerator::~ReachabilityStanceGenerator()
{

}


bool ReachabilityStanceGenerator::generateStance(const craftsman_msgs::GenerateStance::Request &req,
						 std::vector<geometry_msgs::PoseStamped> &stances)
{
  ROS_INFO_STREAM("ReachabilityStanceGenerator::generateStance -- gonna do something for " << robot_name_ << "!");

  if(!inv_init_)
  {
    ROS_ERROR("Inverse Reachability NOT loaded. Exit and not trying anything");
    return false;
  }
  
  geometry_msgs::PoseStamped tmp, avg;
  avg.header.frame_id = tmp.header.frame_id = "world";
  std::string err_msg;
  // average the ee_poses, transform them to world and place a stance at zero z

  if(req.ee_poses.size() == 0)
    return false;
  
  std::vector<geometry_msgs::PoseStamped> grasp_poses;
  for (auto &ee : req.ee_poses)
  {
    try
    {
      ros::Time start_time = ros::Time::now();
      while (!tf_.waitForTransform(ee.header.frame_id, tmp.header.frame_id, ros::Time::now(),
                                   ros::Duration(1.0), ros::Duration(0.01), &err_msg))
      {
        ROS_ERROR_STREAM("ReachabilityStanceGenerator::generateStance() -- No transform from " <<
                         ee.header.frame_id << " to " << tmp.header.frame_id << ".  Error: " <<
                         err_msg);
        err_msg = "";
        if (ros::Time::now() - start_time > ros::Duration(5.0))
        {
          ROS_ERROR_STREAM("NaiveStanceGenerator::generateStance() -- transform gave up after timeout of 5 seconds");
          return false;
        }
      }
      // transform the ee to world
      tf_.transformPose(tmp.header.frame_id, ee, tmp);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_ERROR("NaiveStanceGenerator::generateStance() -- trouble transforming pose from %s to %s. TransformException: %s", ee.header.frame_id.c_str(), tmp.header.frame_id.c_str(), ex.what());
      return false;
    }

    grasp_poses.push_back(tmp);
  }

  std::vector<std::pair<geometry_msgs::Point, std::vector<geometry_msgs::Pose> > > basePoses;
  associatePose(basePoses,
		grasp_poses,
		winv_);

  ros::Time ti = ros::Time::now();
  printf("Base poses: %lu ......\n", basePoses.size());
  for(auto gi : basePoses)
  {
    geometry_msgs::PoseStamped ps;
    
    ps.pose.position = gi.first;
    if(gi.second.size() == 0)
      continue;
    ps.pose.orientation = gi.second[0].orientation;
    ps.header.frame_id = grasp_poses[0].header.frame_id;
    ps.header.stamp = ti;
    stances.push_back(ps);
  }
  
  return true;
}

/***
 *
 */
void ReachabilityStanceGenerator::associatePose(std::vector<std::pair<geometry_msgs::Point, std::vector<geometry_msgs::Pose> > >& basePoses,
						const std::vector<geometry_msgs::PoseStamped> & grasp_poses,
						const reachability_msgs::WorkSpace& winv)
{
  // create a point cloud which consists of all of the possible base locations for all grasp poses and a list of base
  // pose orientations
  printf("Associate pose: Resolution: %f num spheres: %lu \n",
	 winv.resolution,
	 winv.WsSpheres.size());
  std::vector<geometry_msgs::Pose> all_poses;

  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
  double min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = 10000; min_y = 10000; min_z = 10000;
  max_x = -10000; max_y = -10000; max_z = -10000;
  
  for (int i = 0; i < grasp_poses.size(); ++i)
  {
    // Grasp pose in fixed_frame
    tf2::Transform tf_grasp;
    tf2::fromMsg(grasp_poses[i].pose, tf_grasp);
    
    for(int j = 0; j <winv.WsSpheres.size(); ++j)
    {
      for(int k = 0; k < winv.WsSpheres[j].poses.size(); ++k)
      {
	tf2::Transform tf_wis;
	tf2::fromMsg(winv.WsSpheres[j].poses[k], tf_wis);
	tf2::Transform tf_ref_frame;
	geometry_msgs::Pose pose_ref_frame;
	tf_ref_frame = tf_grasp*tf_wis;
	tf2::toMsg(tf_ref_frame, pose_ref_frame);
	all_poses.push_back(pose_ref_frame); // trns_col

	pcl::PointXYZ point;
	point.x = pose_ref_frame.position.x;
	point.y = pose_ref_frame.position.y;
	point.z = pose_ref_frame.position.z;
	if(point.x < min_x) min_x = point.x;
	if(point.y < min_y) min_y = point.y;
	if(point.z < min_z) min_z = point.z;
	if(point.x > max_x) max_x = point.x;
	if(point.y > max_y) max_y = point.y;
	if(point.z > max_z) max_z = point.z;
	cloud->push_back(point);
      }
    }
    
  }  // done creating base pose cloud

  // Create octree for binning the base poses
  double resolution = winv.resolution;    
  pcl::octree::OctreePointCloudSearch< pcl::PointXYZ > octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  // 
  unsigned char maxDepth = 16;
  sphere_discretization::SphereDiscretization sd;
  octomap::point3d pmin, pmax;
  pmin.x() = min_x; pmin.y() = min_y; pmin.z() = min_z;
  pmax.x() = max_x; pmax.y() = max_y; pmax.z() = max_z;
  octomap::OcTree* tree = sd.generateBoxTree(pmin, pmax, resolution);
  std::vector< octomap::point3d > spCenter;
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(maxDepth), end = tree->end_leafs(); it != end; ++it)
    spCenter.push_back(it.getCoordinate());

  
  // add all base poses from cloud to an octree
  for (int i = 0; i < spCenter.size(); i++)
  {
    pcl::PointXYZ searchPoint;
    searchPoint.x = spCenter[i].x();
    searchPoint.y = spCenter[i].y();
    searchPoint.z = spCenter[i].z();

    // Find all base poses that lie in the given voxel
    std::vector< int > pointIdxVec;
    octree.voxelSearch(searchPoint, pointIdxVec);
    if (pointIdxVec.size() > 0)
    {
      geometry_msgs::Point p;
      p.x = searchPoint.x;
      p.y = searchPoint.y;
      p.z = searchPoint.z;
      
      // For a given voxel, add all base poses to the multimap for later retreival
      std::vector<geometry_msgs::Pose> gis;
      for (size_t j = 0; j < pointIdxVec.size(); ++j)
	gis.push_back(all_poses[pointIdxVec[j]]);
      
      basePoses.push_back(std::make_pair(p, gis));
    }
  }

  printf("Base poses inside the function: %lu ****  \n", basePoses.size());
}

PLUGINLIB_EXPORT_CLASS(reachability_stance_generators::ReachabilityStanceGenerator, stance_generator_base::StanceGeneratorBase);  // NOLINT
