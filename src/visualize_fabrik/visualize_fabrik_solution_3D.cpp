#include <ros/ros.h>
#include <ros/console.h>

#include <cmath>
#include <vector>

#include <fabrik/util/math.h>
#include <fabrik/base/fabrik.h>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rvt = rviz_visual_tools;

void publishLabelHelper(rvt::RvizVisualToolsPtr& visual_tools_, const Eigen::Isometry3d& pose, const std::string& label)
{
  Eigen::Vector3d location = pose.translation();
  std::string str = label + ":\n" + std::to_string(location(0)) + "\n" + std::to_string(location(1)) + "\n" + std::to_string(location(2));
  Eigen::Isometry3d pose_copy = pose;
  pose_copy.translation().x() -= 0.2;
  visual_tools_->publishText(pose_copy, str, rvt::WHITE, rvt::scales::LARGE, false);
}

void removeMarker(ros::NodeHandle& node_handle, int id_to_remove)
{
  // to delete markers, publish to the /rviz_visual_tools topic. I should remember the id and set the action to 3 I guess
  ros::Publisher pub_marker = node_handle.advertise<visualization_msgs::MarkerArray>("/rviz_visual_tools", 10);
  visualization_msgs::Marker empty_marker;
  empty_marker.header.frame_id = "world";
  empty_marker.id = id_to_remove;
  empty_marker.header.stamp = ros::Time();
  empty_marker.ns = "deleteAllMarkers";  
  empty_marker.action = 3; // action 3 means delete
  empty_marker.pose.orientation.w = 1;

  visualization_msgs::MarkerArray empty_marker_array;
  empty_marker_array.markers.push_back(empty_marker);
  pub_marker.publish(empty_marker_array);
}
  
int main(int argc, char** argv)
{
  const std::string NODE_NAME = "visualize_fabrik_solution";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  //------------------------ make a 3d robot:
  fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot3D();

  // ---------------------- Solve another forward kinematics close to the first one:
  fabrik::RobotStatePtr robot_state_1 = std::make_shared<fabrik::RobotState>(robot_model);
  robot_state_1->setReachingDirection(fabrik::ReachingDirection::FORWARD);
  double theta_1 = M_PI_4 / 5;
  double theta_2 = 0.2;
  double theta_3 = 0.41;
  std::vector<double> fk_joints_values_1 = {theta_1, theta_2, theta_3};
  for (int k = 0; k < 3; ++k)
      robot_state_1->updateState(fk_joints_values_1[k], k);

  Eigen::Affine3d end_effector_1 = robot_state_1->getFrames(2).second;
  robot_state_1->printState("visualize_fabrik_solution_3D: first configuration", std::vector<int>{0,1,2});


  fabrik::RobotStatePtr robot_state_2 = std::make_shared<fabrik::RobotState>(robot_model);   
  robot_state_2->setReachingDirection(fabrik::ReachingDirection::FORWARD);
  theta_1 = theta_1 + 0.4; // 0.2;
  theta_2 = theta_2 + 0.3; // 0.2;
  theta_3 = theta_3 + 0.2; // 0.1;
  std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
  for (int k = 0; k < 3; ++k)
      robot_state_2->updateState(fk_joints_values_2[k], k);

  robot_state_2->printState("FABRIK - second configuration", std::vector<int>{-1});

  Eigen::Affine3d end_effector_2 = robot_state_2->getFrames(2).second;

  Eigen::Affine3d target = end_effector_2;
  double threshold = 0.001;
  double requested_iteration_num = 15;

  fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(robot_model, fk_joints_values_1));

  fabrik->setInverseKinematicsInput(target,
                                    threshold,
                                    requested_iteration_num,
                                    fabrik::CalculatorType::POSITION);
    
  fabrik::IKOutput output;
  bool solved = fabrik->solveIK(output);

  if(solved)
  {
    std::cout << "---------- error track ----------" << std::endl;
    for(int i = 0; i < output.target_ee_error_track.size(); ++i)
    {
      std::cout << output.target_ee_error_track[i] << std::endl;
    }
  }

  std::cout << "output.start_to_aim.size() ================= " << output.start_to_aim_track.size() << std::endl;


  // Visualization
  // ========================================================================================
    
    rvt::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    ROS_INFO("Sleeping 3 seconds before running demo");
    ros::Duration(3.0).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

  // --------------------------------------------------
  visual_tools_->prompt("press next to see the Initial pose of the End_Effector");
  
  Eigen::Isometry3d coordinate_isometry;
  coordinate_isometry.translation() = end_effector_1.translation();
  coordinate_isometry.linear() = end_effector_1.rotation();
  visual_tools_->publishAxis(coordinate_isometry);
  publishLabelHelper(visual_tools_, coordinate_isometry, "Initial Pose");
  visual_tools_->trigger();

  for (int k = 0; k < 3; ++k)
    {   
      // link
      Eigen::Vector3d point1 = robot_state_1->getFrames(k).first.translation();
      Eigen::Vector3d point2 = robot_state_1->getFrames(k).second.translation();
      visual_tools_->publishCylinder(point1, point2);   

      Eigen::Isometry3d coordinate_isometry;
      coordinate_isometry.translation() = robot_state_1->getFrames(k).first.translation();
      coordinate_isometry.linear() = robot_state_1->getFrames(k).first.rotation();
      visual_tools_->publishAxis(coordinate_isometry);
      visual_tools_->trigger();

      coordinate_isometry.translation() = robot_state_1->getFrames(k).second.translation();
      coordinate_isometry.linear() = robot_state_1->getFrames(k).second.rotation();
      visual_tools_->publishAxis(coordinate_isometry);
      visual_tools_->trigger();
    }

  // visual_tools_->prompt("press next to delete the first marker");
  // removeMarker(node_handle, 1);
  // visual_tools_->removeMarkerWithID(1);
 
  // --------------------------------------------------
    visual_tools_->prompt("press next to see the Taregt for Inverse Kinematics");
    
    coordinate_isometry.translation() = target.translation();
    coordinate_isometry.linear() = target.rotation();
    visual_tools_->publishAxis(coordinate_isometry);
    publishLabelHelper(visual_tools_, coordinate_isometry, "target");
    visual_tools_->trigger();

  // --------------------------------------------------
  visual_tools_->prompt("press next to see the procedure");
  
  // *** start_to_aim(green) and start_to_aim_project(brown) must be on the same plane that the link(blue) lies on ***

  int bfr = output.frames_matrix.size();
  for (int t = 0; t < bfr; ++t)
  {
    if( std::remainder(t, 2) == 0 )
    {
      for (int k = 2; k > -1; --k)
      {   
        // link
        Eigen::Vector3d point1 = output.frames_matrix[t][k].first.translation();
        Eigen::Vector3d point2 = output.frames_matrix[t][k].second.translation();
        visual_tools_->publishCylinder(point1, point2);   

        // in backward reaching there is no calculate and projection for J_0
        // if ( k != 0)
        // {
        //   Eigen::Vector3d point1 = output.frames_matrix[t][k].first.translation();
        //   Eigen::Vector3d point2 = point1 + output.start_to_aim_track[t][k];
        //   visual_tools_->publishCylinder(point1, point2, rvt::colors::GREEN, rvt::scales::SMALL, "start_to_aim");
        //   Eigen::Isometry3d text_pose; 
        //   text_pose.translation() = point1;
        //   text_pose.linear() = target.rotation();
        //   publishLabelHelper(visual_tools_, text_pose, "s2" );
        //   text_pose.translation() = point2;
        //   publishLabelHelper(visual_tools_, text_pose, "e0" );
          
        //   point2 = point1 + output.start_to_aim_projected_track[t][k];
        //   visual_tools_->publishCylinder(point1, point2, rvt::colors::BROWN, rvt::scales::SMALL, "start_to_aim_projected"); 
        //   text_pose.translation() = point2;
        //   publishLabelHelper(visual_tools_, text_pose, "e0_projected" );

        //   point2 = point1 + output.start_to_end_track[t][k];
        //   visual_tools_->publishCylinder(point1, point2, rvt::colors::WHITE, rvt::scales::SMALL, "start_to_end");
        //   text_pose.translation() = point2;
        //   publishLabelHelper(visual_tools_, text_pose, "s1" );

        //   point2 = point1 + output.start_to_end_projected_track[t][k];
        //   visual_tools_->publishCylinder(point1, point2, rvt::colors::CYAN, rvt::scales::SMALL, "start_to_end_projected");
        //   text_pose.translation() = point2;
        //   publishLabelHelper(visual_tools_, text_pose, "s1_projected" );
        // }
        
        // first
        Eigen::Isometry3d c_isometry;
        c_isometry.translation() = output.frames_matrix[t][k].first.translation();
        c_isometry.linear() = output.frames_matrix[t][k].first.rotation();
        visual_tools_->publishAxis(c_isometry);
        // publishLabelHelper(visual_tools_, coordinate_isometry, "start" + std::to_string(k));
        visual_tools_->trigger();

        c_isometry.translation() = output.frames_matrix[t][k].second.translation();
        c_isometry.linear() = output.frames_matrix[t][k].second.rotation();
        visual_tools_->publishAxis(c_isometry);
        publishLabelHelper(visual_tools_, c_isometry, "end" + std::to_string(k));
        visual_tools_->trigger();

        ros::Duration(1.0).sleep();
        // visual_tools_->prompt("next");
      }
    }else
    {
      for (int k = 0; k < 3; ++k)
      {   
        // link
      Eigen::Vector3d point1 = output.frames_matrix[t][k].first.translation();
      Eigen::Vector3d point2 = output.frames_matrix[t][k].second.translation();
      visual_tools_->publishCylinder(point1, point2);   

      // visual_tools_->publishCylinder(output.frames_matrix[t][k].first.translation(),
      //                                 output.frames_matrix[t][k].first.translation() + output.start_to_aim_track[t][k],
      //                                 rvt::colors::GREEN, rvt::scales::SMALL, "start_to_aim");
      
      // visual_tools_->publishCylinder(output.frames_matrix[t][k].first.translation(),
      //                                 output.frames_matrix[t][k].first.translation() + output.start_to_aim_projected_track[t][k],
      //                                 rvt::colors::BROWN, rvt::scales::SMALL, "start_to_aim_projected"); 

      // visual_tools_->publishCylinder(output.frames_matrix[t][k].first.translation(),
      //                                 output.frames_matrix[t][k].first.translation() + output.start_to_end_track[t][k],
      //                                 rvt::colors::WHITE, rvt::scales::SMALL, "start_to_end");

      // visual_tools_->publishCylinder(output.frames_matrix[t][k].first.translation(),
      //                               output.frames_matrix[t][k].first.translation() + output.start_to_end_projected_track[t][k],
      //                               rvt::colors::CYAN, rvt::scales::SMALL, "start_to_end_projected");

        // first
        Eigen::Isometry3d coordinate_isometry;
        coordinate_isometry.translation() = output.frames_matrix[t][k].first.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].first.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        // publishLabelHelper(visual_tools_, coordinate_isometry, "start" + std::to_string(k));
        visual_tools_->trigger();

        coordinate_isometry.translation() = output.frames_matrix[t][k].second.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].second.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        // publishLabelHelper(visual_tools_, coordinate_isometry, "end" + std::to_string(k));
        visual_tools_->trigger();

        ros::Duration(1.0).sleep();
       // visual_tools_->prompt("next");
      } 
    }  
    
  }

    

  visual_tools_->prompt("done");

return 0;
}


