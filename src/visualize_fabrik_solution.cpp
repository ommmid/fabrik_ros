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
    Eigen::Isometry3d pose_copy = pose;
    pose_copy.translation().x() -= 0.2;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
}

int main(int argc, char** argv)
{
  const std::string NODE_NAME = "visualize_fabrik_solution";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // ros::NodeHandle node_handle("~");

  std::cout << "--------- test " << std::endl;

  double randm_num = fabrik::randomDouble(0, 1);
  std::cout << "random num from fabrik: " << randm_num << std::endl;

  // ---------------------- Solve another forward kinematics close to the first one:
  fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();

  fabrik::RobotStatePtr robot_state_1 = std::make_shared<fabrik::RobotState>(robot_model);
  robot_state_1->setReachingDirection(fabrik::ReachingDirection::FORWARD);
  double theta_1 = M_PI_4;
  double theta_2 = 0.1;
  double theta_3 = 0.1;
  std::vector<double> fk_joints_values_1 = {theta_1, theta_2, theta_3};
  for (int k = 0; k < 3; ++k)
      robot_state_1->updateState(fk_joints_values_1[k], k);

  Eigen::Affine3d end_effector_1 = robot_state_1->getFrames(2).second;
  robot_state_1->printState("FABRIK - first configuration", std::vector<int>{-1});


  fabrik::RobotStatePtr robot_state_2 = std::make_shared<fabrik::RobotState>(robot_model);   
  robot_state_2->setReachingDirection(fabrik::ReachingDirection::FORWARD);
  theta_1 = M_PI_4 + 0.1;
  theta_2 = 0.1;
  theta_3 = 0.1;
  std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
  for (int k = 0; k < 3; ++k)
      robot_state_2->updateState(fk_joints_values_2[k], k);

  robot_state_2->printState("FABRIK - second configuration", std::vector<int>{-1});

  Eigen::Affine3d end_effector_2 = robot_state_2->getFrames(2).second;

  Eigen::Affine3d target = end_effector_2;
  double threshold = 0.001;
  double requested_iteration_num = 3;

  fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(robot_model,
                                              fk_joints_values_1,
                                              target,
                                              threshold,
                                              requested_iteration_num,
                                              fabrik::CalculatorType::POSITION));

  fabrik::FabrikOutput output;
  bool solved = fabrik->solve(output);

  std::cout << "solved? " << solved << std::endl;
  if(solved)
  {
      std::cout << "total iteration: " << output.final_iteration_num << std::endl;
      std::cout << "error: " << output.target_ee_error << std::endl;
      for (int k = 0; k < 3; ++k)
          std::cout << "joint value_" << k << ":" << output.solution_joints_values[k] << std::endl;
  }

  // std::vector<std::vector<std::pair<Eigen::Affine3d, Eigen::Affine3d>>> frames_matrix;
  // output.frames_matrix


  // convert affine and isometry
  // Eigen::AffineCompact3f a;
  // Eigen::Isometry3f b;
  // b.translation() = a.translation();
  // b.linear() = a.rotation();

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
      }
  // --------------------------------------------------
    visual_tools_->prompt("press next to see the Taregt for Inverse Kinematics");
    
    coordinate_isometry.translation() = target.translation();
    coordinate_isometry.linear() = target.rotation();
    visual_tools_->publishAxis(coordinate_isometry);
    publishLabelHelper(visual_tools_, coordinate_isometry, "target");
    visual_tools_->trigger();

  // --------------------------------------------------
  visual_tools_->prompt("press next to see the procedure");
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

        // first
        Eigen::Isometry3d coordinate_isometry;
        coordinate_isometry.translation() = output.frames_matrix[t][k].first.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].first.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        publishLabelHelper(visual_tools_, coordinate_isometry, "start" + std::to_string(k));
        visual_tools_->trigger();

        coordinate_isometry.translation() = output.frames_matrix[t][k].second.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].second.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        publishLabelHelper(visual_tools_, coordinate_isometry, "end" + std::to_string(k));
        visual_tools_->trigger();

        ros::Duration(1.0).sleep();
      }
    }else
    {
      for (int k = 0; k < 3; ++k)
      {   
        // link
        Eigen::Vector3d point1 = output.frames_matrix[t][k].first.translation();
        Eigen::Vector3d point2 = output.frames_matrix[t][k].second.translation();
        visual_tools_->publishCylinder(point1, point2);   

        // first
        Eigen::Isometry3d coordinate_isometry;
        coordinate_isometry.translation() = output.frames_matrix[t][k].first.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].first.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        publishLabelHelper(visual_tools_, coordinate_isometry, "start" + std::to_string(k));
        visual_tools_->trigger();

        coordinate_isometry.translation() = output.frames_matrix[t][k].second.translation();
        coordinate_isometry.linear() = output.frames_matrix[t][k].second.rotation();
        visual_tools_->publishAxis(coordinate_isometry);
        publishLabelHelper(visual_tools_, coordinate_isometry, "end" + std::to_string(k));
        visual_tools_->trigger();

        ros::Duration(1.0).sleep();
      } 
    }
    

    
  }

  visual_tools_->prompt("done");

return 0;
}


