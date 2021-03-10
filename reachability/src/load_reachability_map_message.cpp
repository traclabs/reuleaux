#include <ros/ros.h>
#include <ros/package.h>

#include <reachability/helpers.h>
#include <reachability_msgs/WorkSpace.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "load_reachability_map_message");
  ros::NodeHandle pnh_("~");
  ros::NodeHandle nh_;

  ros::Publisher workspace_pub = nh_.advertise< reachability_msgs::WorkSpace >("reachability_map", 1);
  ros::Rate loop_rate(10);
  
  int count = 0;

  std::string reachability_map_;
  std::string reachability_fullname_;
  std::string reference_frame_;

  std::string path(ros::package::getPath("reachability") + "/maps/");
  
  pnh_.getParam("reachability_map", reachability_map_);
  pnh_.getParam("reference_frame", reference_frame_);

  reachability_fullname_ = path + reachability_map_;

  reachability_msgs::WorkSpace ws;
  if(!loadReachabilityMapMessage(reachability_fullname_,
				 ws))
    return 2;
  
   
  while (ros::ok())
  {
    workspace_pub.publish(ws);

    ros::spinOnce();
    sleep(5);
    ++count;
  }

  return 0;
}
