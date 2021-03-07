/**
 * @file debug_visualize_poses.cpp
 */
#include <reachability/sphere_discretization.h>
#include <geometry_msgs/TransformStamped.h>
#include <octomap/math/Utils.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @function main
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "debug_visualize_poses");
  ros::NodeHandle nh;

  tf2_ros::TransformBroadcaster br;
  octomap::point3d origin(0, 0, 0);
  double radius = 2.0;
  std::vector<geometry_msgs::Pose> poses;
  printf("Start?\n");
  sphere_discretization::SphereDiscretization sd;
  sd.make_sphere_poses(origin, radius,
		       poses);

  // Visualize
  std::string reference_frame = "world";

  ros::Rate r(1.0);
  while(ros::ok())
  {
    for(int i = 0; i < poses.size(); ++i)
    {
      geometry_msgs::TransformStamped tfs;
      tfs.header.stamp = ros::Time::now();
      tfs.header.frame_id = reference_frame;
      tfs.child_frame_id = "debug_tf_" + std::to_string(i);
      tfs.transform.translation.x = poses[i].position.x;
      tfs.transform.translation.y = poses[i].position.y;
      tfs.transform.translation.z = poses[i].position.z;
      tfs.transform.rotation.x = poses[i].orientation.x;
      tfs.transform.rotation.y = poses[i].orientation.y;
      tfs.transform.rotation.z = poses[i].orientation.z;
      tfs.transform.rotation.w = poses[i].orientation.w;
      br.sendTransform(tfs);
    }

    r.sleep();
    ros::spinOnce();
    
  } // end while

  return 0;
}
