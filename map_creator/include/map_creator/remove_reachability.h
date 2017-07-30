#ifndef REMOVE_REACHABILITY_H
#define REMOVE_REACHABILITY_H

#include<chrono>
#include<ros/ros.h>
#include<moveit_msgs/GetPlanningScene.h>

#include<octomap/octomap.h>
#include<octomap_msgs/conversions.h>
#include<octomap/AbstractOcTree.h>

#include<pcl/point_cloud.h>
#include<pcl/octree/octree.h>

#include<map_creator/WorkSpace.h>


#define SPIN_RATE 10

class remove_obstacles_reachability
{
public:

  enum filterType{
    VOXEL,
    INSCRIBED_SPHERE,
    CIRCUMSCRIBED_SPHERE //conservative, default
  };

  remove_obstacles_reachability();
  ~remove_obstacles_reachability();
  void spin(filterType filter_type);


private:
  void readMap(const map_creator::WorkSpace msg);
  void createObstaclesPointCloud(octomap::OcTree& tree, pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_vertices);
  void createFilteredReachability(filterType type, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& search_tree, map_creator::WorkSpace& filtered_map, map_creator::WorkSpace& colliding_map);

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_reachability_;
  ros::Publisher pub_filtered_reachability_;
  ros::Publisher pub_colliding_reachability_;
  ros::ServiceClient client_get_planning_scene_;
  moveit_msgs::GetPlanningScene scene_srv_;
  map_creator::WorkSpace reachability_map_;
  double reachability_resolution_;

};


#endif // REMOVE_REACHABILITY_H
