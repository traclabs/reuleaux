#include <ros/ros.h>
#include <ros/package.h>

#include <reachability_msgs/WorkSpace.h>
#include <reachability/hdf5_dataset.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "workspace");
  ros::NodeHandle pnh_("~");
  ros::NodeHandle nh_;

  // TODO: It can be published as a latched topic. So the whole message will be published just once and stay on the
  // topic
  ros::Publisher workspace_pub = nh_.advertise< reachability_msgs::WorkSpace >("reachability_map", 1);
  // bool latchOn = 1;
  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1, latchOn);
  ros::Rate loop_rate(10);
  
  int count = 0;

  std::string reachability_map_;
  std::string reachability_fullname_;
  std::string reference_frame_;

  std::string path(ros::package::getPath("reachability") + "/maps/");
  
  pnh_.getParam("reachability_map", reachability_map_);
  pnh_.getParam("reference_frame", reference_frame_);

  reachability_fullname_ = path + reachability_map_ + std::string(".h5");

  hdf5_dataset::Hdf5Dataset h5(reachability_fullname_);
  h5.open();

  MultiMapPtr pose_col_filter;
  MapVecDoublePtr sphere_col;
  float res;
  h5.loadMapsFromDataset(pose_col_filter, sphere_col, res);

  // Creating messages
  reachability_msgs::WorkSpace ws;
  ws.header.stamp = ros::Time::now();
  ws.header.frame_id = reference_frame_;
  ws.resolution = res;

  for (MapVecDoublePtr::iterator it = sphere_col.begin(); it != sphere_col.end(); ++it)
  {
    reachability_msgs::WsSphere wss;
    wss.point.x = (*it->first)[0];
    wss.point.y = (*it->first)[1];
    wss.point.z = (*it->first)[2];
    wss.ri = it->second;
    
    for (MultiMapPtr::iterator it1 = pose_col_filter.lower_bound(it->first); it1 != pose_col_filter.upper_bound(it->first); ++it1)
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
    ws.WsSpheres.push_back(wss);
  }
  
  while (ros::ok())
  {
    workspace_pub.publish(ws);

      ros::spinOnce();
      sleep(5);
      ++count;
    }

  return 0;
}
