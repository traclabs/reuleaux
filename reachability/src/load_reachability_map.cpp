#include <ros/ros.h>
#include <ros/package.h>

#include <reachability/helpers.h>
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

  reachability_msgs::WorkSpace ws;
  if(!readReachMapToRosMessage(reachability_fullname_,
			       reference_frame_,
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
