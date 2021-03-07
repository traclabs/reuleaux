/**
 * @file create_reachability_map
 * @brief Create Reachability Map, duh
 */
// The spheres and poses are fused in a single dataset, instead of
// two datasets for sphere and poses
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <reachability/sphere_discretization.h>
#include <reachability/robot_kinematics.h>
#include <reachability/hdf5_dataset.h>
#include <boost/format.hpp>


//struct stat st;

typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;
typedef std::vector<std::vector<double> > VectorOfVectors;

/**
 * @function main
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "create_reachability_map");
  ros::NodeHandle pnh("~");
  ros::Time startit = ros::Time::now();

  std::string path(ros::package::getPath("reachability") + "/maps/");
  std::string file_;
  std::string filename_;

  float resolution_ = 0.05;
  
  pnh.getParam("resolution", resolution_);
  pnh.getParam("filename", file_);
  filename_ = path + file_;

  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("workspace", 10);
  ros::Rate loop_rate(10);  
  int count = 0;


  
  while (ros::ok())
  {
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    sphere_discretization::SphereDiscretization sd;
    float r = 1;
    // This point will be the base of the robot
    octomap::point3d origin = octomap::point3d(0, 0, 0); 
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution_);
    std::vector< octomap::point3d > node_coordinates;
    
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution_);
    int sphere_count = tree->getNumLeafNodes();
    node_coordinates.reserve(sphere_count);
    
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth),
	   end = tree->end_leafs();
	 it != end; ++it)
    {
      node_coordinates.push_back(it.getCoordinate());
    }

    ROS_INFO("Total no of spheres now: %lu ", node_coordinates.size());
    ROS_INFO("Please hold ON...");

    // If the resolution is 0.01 the programs not responds
    float radius = resolution_;

    VectorOfVectors sphere_coord;
    sphere_coord.resize( node_coordinates.size() );

    MultiVector pose_col;
    pose_col.reserve( node_coordinates.size() * sd.get_sphere_num_pose_divisions());

    for (int i = 0; i < node_coordinates.size(); i++)
    {
      static std::vector< geometry_msgs::Pose > pose;
      sd.convertPointToVector(node_coordinates[i], sphere_coord[i]);

      sd.make_sphere_poses(node_coordinates[i], radius, pose);
      for (int j = 0; j < pose.size(); j++)
      {
        static std::vector< double > point_on_sphere;
        sd.convertPoseToVector(pose[j], point_on_sphere);
        pose_col.push_back( std::make_pair(point_on_sphere, &sphere_coord[i]));
      }
    }

    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are stored.
    // Only the First joint solution is stored. 
    // TODO Support for more than 6DOF robots needs 
    RobotKinematics rk(pnh);
    if(!rk.init())
      return 1;
    
    MultiMapPtr pose_col_filter;
    VectorOfVectors ik_solutions;
    ik_solutions.reserve( pose_col.size() );

    ROS_WARN("Pose_col size: %lu ", pose_col.size());
    for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {
      if(count % 2000 == 0)
	ROS_WARN("Count: %lu/%lu",
		 count,
		 pose_col.size());
      
      //static std::vector< double > joints(6);
      std::vector< double > joints;
      int solns;
      if (rk.isIKSuccess(it->first, joints, solns))
      {
        pose_col_filter.insert( std::make_pair( it->second, &(it->first)));
        ik_solutions.push_back(joints);
      }
      count++;
    }

    ROS_INFO("Total number of poses: %lu", pose_col.size());
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)
    MapVecDoublePtr sphere_color;


    for (MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      const std::vector<double>* sphere_coord    = it->first;
      //const std::vector<double>* point_on_sphere = it->second;

      // Reachability Index D=R/N*100;
      float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / node_coordinates.size()) * 100;
      sphere_color.insert( std::make_pair(it->first, double(d)));
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

    // Creating maps now. Saving map to dataset
    hdf5_dataset::Hdf5Dataset h5(filename_);
    h5.saveReachMapsToDataset(pose_col_filter, sphere_color, resolution_);

    double dif = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Elasped time is %.2lf seconds.", dif);
    ROS_INFO("Completed");
    ros::spinOnce();
    // sleep(10000);
    return 1;
    loop_rate.sleep();
    
  }
  return 0;
}
