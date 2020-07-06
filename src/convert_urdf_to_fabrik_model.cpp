#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>

#include "fabrik_ros/fabrik_model.h"
#include <fabrik/base/fabrik.h>

#include <boost/function.hpp>


int main(int argc, char** argv)
{
  const std::string NODE_NAME = "urdf2fabrik";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

   FabrikModel fabrik_model;

// from a file
//  std::string urdf_file = "urdf/my_robot.urdf";
//   urdf::Model model;
//   if (!model.initFile(urdf_file)){
//     ROS_ERROR("Failed to parse urdf file");
//     return -1;
//   }
//   ROS_INFO("Successfully parsed urdf file");


//    ROS_INFO_STREAM( "model name: " << model.getName());
//    ROS_INFO_STREAM( "root: " <<       model.getRoot()->name);

//     std::string link_name = "link1";
//     urdf::LinkConstSharedPtr link_ = model.getLink(link_name);

//     ROS_INFO_STREAM( "===>>>: " << model.links_.size());
//     ROS_INFO_STREAM( "name ===>>>: " << model.links_["panda_link0"]->name);


// from rostopic

return 0;
}