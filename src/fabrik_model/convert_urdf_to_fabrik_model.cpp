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

    // from a file
    // std::string urdf_file = "/home/omid/ws_fabrik_ros/src/fabrik_ros/urdf/my_robot.urdf";

    // convert the xacro to urdf by: rosrun xacro xacro [the xacro file]] > model.urdf
    std::string urdf_file = "/home/omid/ws_fabrik_ros/src/fabrik_ros/urdf/model.urdf";
    urdf::Model model;
    if (!model.initFile(urdf_file)){
        ROS_ERROR("Failed to parse urdf file");
    return -1;
    }
    ROS_INFO("Successfully parsed urdf file");

    ROS_INFO_STREAM( "===> model name: " << model.getName());
    ROS_INFO_STREAM( "===> root: " <<       model.getRoot()->name);

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);

    ROS_INFO_STREAM( "============= links =================" );
    for(auto link : links)
    {
        std::string link_name = link->name;
        ROS_INFO_STREAM( "link ===> " << link_name );
        std::vector<urdf::JointSharedPtr> joints = link->child_joints; 
        for (int j = 0; j < joints.size() == 1; ++j)
        {
            ROS_INFO_STREAM( "child joint " << j << " ===> " << joints[j]->name );
            ROS_INFO_STREAM( "joint axis ===> " << joints[j]->axis.x << " " << joints[j]->axis.y << " " << joints[j]->axis.z );
            ROS_INFO_STREAM( "joint frame location: " );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.position.x );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.position.y );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.position.z );
            ROS_INFO_STREAM( "joint frame orientation: " );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.rotation.x );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.rotation.y );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.rotation.z );
            ROS_INFO_STREAM( joints[j]->parent_to_joint_origin_transform.rotation.w );

        }
    }

    


    FabrikModel fabrik_model(urdf_file);
    

    // For now, Manually make the links in fabrik. For that, I need to set all the joint values to zero
    // then calculate the relative displacement for each link in rviz
    Eigen::Vector3d vec0(0,0,1);
    vec0.normalize();
    Eigen::Affine3d base(Eigen::AngleAxisd(0, vec0));
    base.translation() = Eigen::Vector3d(0, 0, 0.333/2);

    Eigen::Vector3d vec1(1,0,0);
    vec1.normalize();
    Eigen::Affine3d link1_frame(Eigen::AngleAxisd(-M_PI_2, vec1));
    link1_frame.translation() = Eigen::Vector3d(0, 0, 0.333/2);
    fabrik::Link link1("link1",  link1_frame);

    Eigen::Vector3d vec2(1,0,0);
    vec2.normalize();
    Eigen::Affine3d link2_frame(Eigen::AngleAxisd(M_PI_2, vec2));
    link2_frame.translation() = Eigen::Vector3d(0, -0.316/2, 0);
    fabrik::Link link2("link2",  link2_frame);

    Eigen::Vector3d vec3(1,0,0);
    vec3.normalize();
    Eigen::Affine3d link3_frame(Eigen::AngleAxisd(M_PI_2, vec3));
    link3_frame.translation() = Eigen::Vector3d(0.0825, 0, 0.316/2);
    fabrik::Link link3("link3",  link3_frame);

    Eigen::Vector3d vec4(1,0,0);
    vec4.normalize();
    Eigen::Affine3d link4_frame(Eigen::AngleAxisd(-M_PI_2, vec4));
    link4_frame.translation() = Eigen::Vector3d(-0.0825, 0.384/3, 0);
    fabrik::Link link4("link4",  link4_frame);

    Eigen::Vector3d vec5(1,0,0);
    vec5.normalize();
    Eigen::Affine3d link5_frame(Eigen::AngleAxisd(M_PI_2, vec5));
    link5_frame.translation() = Eigen::Vector3d(0, 0, 0.384);
    fabrik::Link link5("link5",  link5_frame);

    Eigen::Vector3d vec6(1,0,0);
    vec6.normalize();
    Eigen::Affine3d link6_frame(Eigen::AngleAxisd(M_PI_2, vec6));
    link6_frame.translation() = Eigen::Vector3d(0.088, -0.5, 0);
    fabrik::Link link6("link6",  link6_frame);

    Eigen::Vector3d vec7(1,0,0);
    vec7.normalize();
    Eigen::Affine3d link7_frame(Eigen::AngleAxisd(0, vec7));
    link7_frame.translation() = Eigen::Vector3d(0, 0, 1.07);
    fabrik::Link link7("link7",  link7_frame);

return 0;
}