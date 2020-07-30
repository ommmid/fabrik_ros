#include <ros/ros.h>
#include <ros/console.h>
#include <fabrik/util/math.h>

#include <vector>

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
  const std::string NODE_NAME = "fabrik_example";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // ros::NodeHandle node_handle("~");

  std::cout << "--------- test " << std::endl;

  double randm_num = fabrik::randomDouble(0, 1);
  std::cout << "random num from fabrik: " << randm_num << std::endl;

  // ---------------------- Solve another forward kinematics close to the first one:
  fabrik::RobotModelPtr robot_model = fabrik::makeSimpleRobot2D();
  fabrik::RobotStatePtr robot_state_2 = 
      std::make_shared<fabrik::RobotState>(robot_model);
    
  robot_state_2->setReachingDirection(fabrik::ReachingDirection::FORWARD);
  double theta_1 = M_PI_4 + 0.1;
  double theta_2 = 0.1;
  double theta_3 = 0.1;
  std::vector<double> fk_joints_values_2 = {theta_1, theta_2, theta_3};
  for (int k = 0; k < 3; ++k)
      robot_state_2->updateState(fk_joints_values_2[k], k);

  robot_state_2->printState("FABRIK - second configuration", std::vector<int>{-1});

  Eigen::Affine3d end_effector_2 = robot_state_2->getFrames(2).second;

  Eigen::Affine3d target = end_effector_2;
  double threshold = 0.001;
  double requested_iteration_num = 3;

  fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(robot_model, fk_joints_values_2));

  fabrik->setInverseKinematicsInput(target,
                                    threshold,
                                    requested_iteration_num,
                                    fabrik::CalculatorType::POSITION);
 
  fabrik::IKOutput output;
  bool solved = fabrik->solveIK(output);

  std::cout << "solved? " << solved << std::endl;
  if(solved)
  {
      std::cout << "total iteration: " << output.final_iteration_num << std::endl;
      std::cout << "error: " << output.target_ee_error << std::endl;
      for (int k = 0; k < 3; ++k)
          std::cout << "joint value_" << k << ":" << output.solution_joints_values[k] << std::endl;
  }

  // Visualization
  // ========================================================================================
    
    rvt::RvizVisualToolsPtr visual_tools_;
    visual_tools_.reset(new rvt::RvizVisualTools("world", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();  // create publisher before waiting

    ROS_INFO("Sleeping 5 seconds before running demo");
    ros::Duration(5.0).sleep();

    // Clear messages
    visual_tools_->deleteAllMarkers();
    visual_tools_->enableBatchPublishing();

    visual_tools_->prompt("click next to see Cuboid");

    std::string name_ = "fabrik_example";
    ROS_INFO_STREAM_NAMED(name_, "Displaying Rectangular Cuboid");
    double cuboid_max_size = 0.075;
    double cuboid_min_size = 0.01;
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
    double space_between_rows = 0.2;
    double y = 0;
    double step;

    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows;
    pose1.translation().y() = y;
    pose2.translation().y() = y;
    step = 0.1;
    for (double i = 0; i <= 1.0; i += step)
    {
      pose2 = pose1;
      pose2.translation().x() += i * cuboid_max_size + cuboid_min_size;
      pose2.translation().y() += i * cuboid_max_size + cuboid_min_size;
      pose2.translation().z() += i * cuboid_max_size + cuboid_min_size;
      visual_tools_->publishCuboid(pose1.translation(), pose2.translation(), rvt::RAND);

      if (i == 0.0)
      {
        publishLabelHelper(visual_tools_, pose1, "Cuboid");
      }

      pose1.translation().x() += step;
    }
    ROS_INFO_STREAM_NAMED(name_, "====>>>> trigering Rectangular Cuboid");
    visual_tools_->trigger();
    visual_tools_->prompt("done");

return 0;
}


