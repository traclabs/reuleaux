/***
 * @file create_inverse_reachability_map.cpp
 * @brief The inverse reachability map depends on the reachability map. It is an inversion of the poses to the base location
 */
#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include <reachability/sphere_discretization.h>
#include <reachability_msgs/WorkSpace.h>
#include <reachability/helpers.h>

#include <map>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>

#include <string>
#include <time.h>

//////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_inverse_workspace");
  ros::NodeHandle pnh("~");

  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1);
  time_t startit, finish;
  time(&startit);

  std::string file;
  std::string path(ros::package::getPath("reachability") + "/maps/");
  
  std::string reachability_map_;
  std::string reachability_map_fullname_;
  std::string inverse_reach_map_;
  std::string inverse_reach_map_fullname_;
  std::string reference_frame_;
  
  pnh.getParam("reachability_map", reachability_map_);
  pnh.getParam("inverse_reach_map", inverse_reach_map_);
  pnh.getParam("reference_frame", reference_frame_);
  
  reachability_map_fullname_ = path + reachability_map_;
  inverse_reach_map_fullname_ = path + inverse_reach_map_;

  // Load reachability map
  reachability_msgs::WorkSpace ws;
  loadReachabilityMapMessage(reachability_map_fullname_,
			     ws);
  
  ros::Rate loop_rate(10);
  
  // Starting to create the Inverse Reachability map.
  // The resolution will be the same as the reachability map
  unsigned char max_depth = 16;
  unsigned char minDepth = 0;
  float resolution = ws.resolution;
  sphere_discretization::SphereDiscretization sd;

  int total_poses = 0;
  for(int i = 0; i < ws.WsSpheres.size(); ++i)
    total_poses += ws.WsSpheres[i].poses.size();
  
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
  std::vector<geometry_msgs::Pose> trns_col;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  min_x = 10000; min_y = 10000; min_z = 10000;
  max_x = -10000; max_y = -10000; max_z = -10000;
  for(int i = 0; i < ws.WsSpheres.size(); ++i)
  {
    for(int j = 0; j < ws.WsSpheres[i].poses.size(); ++j)
    {
      geometry_msgs::Pose pj, pj_inv;
      tf2::Transform tj, tj_inv;
      pj = ws.WsSpheres[i].poses[j];
      tf2::fromMsg(pj, tj);
      tj_inv = tj.inverse();
      tf2::toMsg(tj_inv, pj_inv);
      
      trns_col.push_back(pj_inv);
      
      pcl::PointXYZ point;
      point.x = pj_inv.position.x;
      point.y = pj_inv.position.y;
      point.z = pj_inv.position.z;
      
      if(pj_inv.position.x < min_x) { min_x = pj_inv.position.x; }
      if(pj_inv.position.y < min_y) { min_y = pj_inv.position.y; }
      if(pj_inv.position.z < min_z) { min_z = pj_inv.position.z; }
      if(pj_inv.position.x > max_x) { max_x = pj_inv.position.x; }
      if(pj_inv.position.y > max_y) { max_y = pj_inv.position.y; }
      if(pj_inv.position.z > max_z) { max_z = pj_inv.position.z; }
      
      cloud->push_back(point);      	
    }
  }

  printf("Min: %f %f %f, max: %f %f %f!! \n",
	 min_x, min_y, min_z,
	 max_x, max_y, max_z);
  octomap::point3d pmin, pmax;
  pmin.x() = min_x; pmin.y() = min_y; pmin.z() = min_z;
  pmax.x() = max_x; pmax.y() = max_y; pmax.z() = max_z;
  octomap::OcTree *tree = sd.generateBoxTree(pmin, pmax, resolution);  
  std::vector< octomap::point3d > node_coordinates;
  
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth),
	 end = tree->end_leafs();
       it != end; ++it)
    node_coordinates.push_back(it.getCoordinate());

  ROS_INFO("Number of poses in RM: %lu (what you loaded)", total_poses);
  ROS_INFO("Number of voxels: %lu (what you intend to fill)", node_coordinates.size());
  
  // As these map is independent of any task points, it is centered around origin.
  // For dependent maps, the whole map will be transformed to that certain task
  // point
  printf("Resolution: %f \n", resolution);
  pcl::octree::OctreePointCloudSearch< pcl::PointXYZ > octree(resolution);
  octree.setInputCloud(cloud);
  octree.addPointsFromInputCloud();

  reachability_msgs::WorkSpace wsi;
  wsi.resolution = resolution;
  wsi.header.frame_id = reference_frame_;

  int skipped = 0;
  for (int i = 0; i < node_coordinates.size(); i++)
  {
    pcl::PointXYZ search_point;
    search_point.x = node_coordinates[i].x();
    search_point.y = node_coordinates[i].y();
    search_point.z = node_coordinates[i].z();
    
    // Neighbors within voxel search
    std::vector< int > point_idx_vec;
    if(search_point.x < min_x - resolution ||
       search_point.x > max_x + resolution ||
       search_point.y < min_y - resolution ||
       search_point.y > max_y + resolution ||
       search_point.z < min_z - resolution ||
       search_point.z > max_z + resolution )
    {
      skipped++;
      continue;
    }
    
    octree.voxelSearch(search_point, point_idx_vec);
    reachability_msgs::WsSphere wss;
    wss.header.frame_id = reference_frame_;
    //wss.ri =
    if (point_idx_vec.size() > 0)
    {
      wss.point.x = search_point.x;
      wss.point.y = search_point.y;
      wss.point.z = search_point.z;
      
      for (size_t j = 0; j < point_idx_vec.size(); ++j)
	wss.poses.push_back(trns_col[point_idx_vec[j]]);
   
      wsi.WsSpheres.push_back(wss);
    }
  }
  printf("Debug point 2 : Skipped: %d / %d = %f \n", skipped, node_coordinates.size(), (double)skipped/(double)node_coordinates.size());
  float dmin, dmax;
  dmin = 1000000;
  dmax = 0;
  std::vector<std::pair<int,double>> sphere_color;
  for (int i = 0; i < wsi.WsSpheres.size(); ++i)
  {
    int pose_size = 50; // This never changes
    double d = (double(wsi.WsSpheres[i].poses.size())) / (double)(50) * 100;
    if( d < dmin )
      dmin = d;
    if( d > dmax )
      dmax = d;
    sphere_color.push_back( std::make_pair(i, double(d)));
  }
  
  ROS_INFO("Number of Spheres in  RM: %lu", ws.WsSpheres.size());
  ROS_INFO("Number of Spheres in IRM: %lu -- %lu", sphere_color.size(), wsi.WsSpheres.size());
  ROS_INFO("Dmin: %f Dmax: %f ", dmin, dmax);
  ROS_INFO("All the poses have Processed. Now saving data to a inverse Reachability Map.");
  saveReachabilityMapMessage(inverse_reach_map_fullname_, wsi);
  
  time(&finish);
  double dif = difftime(finish, startit);
  ROS_INFO("Elapsed time is %.2lf seconds.", dif);
  ROS_INFO("Completed");

  return 0;
}
