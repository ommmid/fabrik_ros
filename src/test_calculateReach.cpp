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
    pose_copy.translation().x() -= 0.4;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XXLARGE, false);
}

void visualizeReach(Eigen::Isometry3d& start_frame_reaching_isometry,
                    Eigen::Isometry3d& end_frame_reaching_isometry,
                    Eigen::Isometry3d& frame_aimed_at_isometry,
                    rvt::RvizVisualToolsPtr& visual_tools_,
                    fabrik::PositionBasedCalculatorPtr& pbc)
{
  visual_tools_->prompt("press next to see things");
  
  double max_plane_size = 0.075;
  double min_plane_size = 0.01;

  visual_tools_->publishXYPlane(start_frame_reaching_isometry, rvt::RED, 1);

  visual_tools_->publishAxis(start_frame_reaching_isometry);
  publishLabelHelper(visual_tools_, start_frame_reaching_isometry, "start_frame_reaching");

  visual_tools_->publishAxis(end_frame_reaching_isometry);
  publishLabelHelper(visual_tools_, end_frame_reaching_isometry, "end_frame_reaching");

  visual_tools_->publishAxis(frame_aimed_at_isometry);
  publishLabelHelper(visual_tools_, frame_aimed_at_isometry, "frame_aimed_at");
  

  visual_tools_->publishCylinder( start_frame_reaching_isometry.translation(),
                                  start_frame_reaching_isometry.translation() + pbc->start_to_aim,
                                  rvt::colors::GREEN, rvt::scales::SMALL, "start_to_aim");

  visual_tools_->publishCylinder( start_frame_reaching_isometry.translation(),
                                  start_frame_reaching_isometry.translation() + pbc->start_to_aim_projected,
                                  rvt::colors::BROWN, rvt::scales::SMALL, "start_to_aim_projected");

  visual_tools_->publishCylinder( start_frame_reaching_isometry.translation(),
                                  start_frame_reaching_isometry.translation() + pbc->start_to_end,
                                  rvt::colors::WHITE, rvt::scales::SMALL, "start_to_end");

  visual_tools_->publishCylinder( start_frame_reaching_isometry.translation(),
                                  start_frame_reaching_isometry.translation() + pbc->start_to_end_projected,
                                  rvt::colors::CYAN, rvt::scales::SMALL, "start_to_end_projected");


}
  
int main(int argc, char** argv)
{
  const std::string NODE_NAME = "visualize_fabrik_solution";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // ---- calculate reach
  fabrik::CalculatorPtr position_based_calculator = std::make_shared<fabrik::PositionBasedCalculator>();

  Eigen::Affine3d start_frame_reaching(Eigen::AngleAxisd(0.30, Eigen::Vector3d(0,0,1)));
  start_frame_reaching.translation() = Eigen::Vector3d(0,0,0);
  std::cout << "link1_frame:\n" << start_frame_reaching.matrix() << std::endl;

  Eigen::Affine3d end_frame_reaching(Eigen::AngleAxisd(0.40, Eigen::Vector3d(3,1,1)));
  end_frame_reaching.translation() = Eigen::Vector3d(1,1,1);
  std::cout << "link2_frame:\n" << end_frame_reaching.matrix() << std::endl;

  Eigen::Affine3d frame_aimed_at(Eigen::AngleAxisd(0.50, Eigen::Vector3d(2,0,1)));
  frame_aimed_at.translation() = Eigen::Vector3d(-2,2,2);
  std::cout << "link3_frame:\n" << frame_aimed_at.matrix() << std::endl;

  position_based_calculator->calculateReach(start_frame_reaching,
                                            end_frame_reaching,
                                            frame_aimed_at);

  fabrik::PositionBasedCalculatorPtr pbc = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);

  Eigen::Isometry3d start_frame_reaching_isometry;
  start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  Eigen::Isometry3d end_frame_reaching_isometry;
  end_frame_reaching_isometry.translation() = end_frame_reaching.translation();
  end_frame_reaching_isometry.linear() = end_frame_reaching.rotation();

  Eigen::Isometry3d frame_aimed_at_isometry;
  frame_aimed_at_isometry.translation() = frame_aimed_at.translation();
  frame_aimed_at_isometry.linear() = frame_aimed_at.rotation();

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

  // ----

  // visualizeReach( start_frame_reaching_isometry,
  //                 end_frame_reaching_isometry,
  //                 frame_aimed_at_isometry,
  //                 visual_tools_,
  //                 pbc);

  // visual_tools_->trigger();
  // visual_tools_->prompt("done");


  // -------------- on the world plane - fine
  // start_frame_reaching = Eigen::AngleAxisd(0.20, Eigen::Vector3d(0,0,1));
  // start_frame_reaching.translation() = Eigen::Vector3d(0.2,0.5,0);

  // start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  // start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  // position_based_calculator->calculateReach(start_frame_reaching,
  //                                           end_frame_reaching,
  //                                           frame_aimed_at);

  // fabrik::PositionBasedCalculatorPtr pbc2 = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);


  //  visualizeReach( start_frame_reaching_isometry,
  //                 end_frame_reaching_isometry,
  //                 frame_aimed_at_isometry,
  //                 visual_tools_,
  //                 pbc2);

  // Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
  //                                   start_frame_reaching.rotation()(1,2),
  //                                   start_frame_reaching.rotation()(2,2));

  // double angle = fabrik::signedAngleBetweenTwoVectors(pbc2->start_to_end_projected, 
  //                                                     pbc2->start_to_aim_projected,
  //                                                     j_plane_normal);

  // std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;

// -------------- at the world origin but different orientation 
  // start_frame_reaching = Eigen::AngleAxisd(0.6, Eigen::Vector3d(1,2,5));
  // start_frame_reaching.translation() = Eigen::Vector3d(0.2,0.5,0);

  // start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  // start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  // position_based_calculator->calculateReach(start_frame_reaching,
  //                                           end_frame_reaching,
  //                                           frame_aimed_at);

  // fabrik::PositionBasedCalculatorPtr pbc2 = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);


  //  visualizeReach( start_frame_reaching_isometry,
  //                 end_frame_reaching_isometry,
  //                 frame_aimed_at_isometry,
  //                 visual_tools_,
  //                 pbc2);

  // Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
  //                                   start_frame_reaching.rotation()(1,2),
  //                                   start_frame_reaching.rotation()(2,2));

  // double angle = fabrik::signedAngleBetweenTwoVectors(pbc2->start_to_end_projected, 
  //                                                     pbc2->start_to_aim_projected,
  //                                                     j_plane_normal);

  // std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;

// -------------- at the world origin but different orientation - fine
  // start_frame_reaching = Eigen::AngleAxisd(0.20, Eigen::Vector3d(1,2,3));
  // start_frame_reaching.translation() = Eigen::Vector3d(0,0,0);

  // start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  // start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  // position_based_calculator->calculateReach(start_frame_reaching,
  //                                           end_frame_reaching,
  //                                           frame_aimed_at);

  // fabrik::PositionBasedCalculatorPtr pbc2 = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);


  //  visualizeReach( start_frame_reaching_isometry,
  //                 end_frame_reaching_isometry,
  //                 frame_aimed_at_isometry,
  //                 visual_tools_,
  //                 pbc2);

  // Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
  //                                   start_frame_reaching.rotation()(1,2),
  //                                   start_frame_reaching.rotation()(2,2));

  // double angle = fabrik::signedAngleBetweenTwoVectors(pbc2->start_to_end_projected, 
  //                                                     pbc2->start_to_aim_projected,
  //                                                     j_plane_normal);

  // std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;

// -------------- at the world origin but different orientation 
  // start_frame_reaching = Eigen::AngleAxisd(0.20, Eigen::Vector3d(0,0,1));
  // start_frame_reaching.translation() = Eigen::Vector3d(0.2,0.5,0.35);

  // start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  // start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  // position_based_calculator->calculateReach(start_frame_reaching,
  //                                           end_frame_reaching,
  //                                           frame_aimed_at);

  // fabrik::PositionBasedCalculatorPtr pbc2 = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);


  //  visualizeReach( start_frame_reaching_isometry,
  //                 end_frame_reaching_isometry,
  //                 frame_aimed_at_isometry,
  //                 visual_tools_,
  //                 pbc2);

  // Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
  //                                   start_frame_reaching.rotation()(1,2),
  //                                   start_frame_reaching.rotation()(2,2));

  // double angle = fabrik::signedAngleBetweenTwoVectors(pbc2->start_to_end_projected, 
  //                                                     pbc2->start_to_aim_projected,
  //                                                     j_plane_normal);

  // std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;

  // ------------  another one
  start_frame_reaching = Eigen::AngleAxisd(0.20, Eigen::Vector3d(1,2,3));
  start_frame_reaching.translation() = Eigen::Vector3d(0.2,0.5,0.35);

  start_frame_reaching_isometry.translation() = start_frame_reaching.translation();
  start_frame_reaching_isometry.linear() = start_frame_reaching.rotation();

  position_based_calculator->calculateReach(start_frame_reaching,
                                            end_frame_reaching,
                                            frame_aimed_at);

  fabrik::PositionBasedCalculatorPtr pbc2 = std::static_pointer_cast<fabrik::PositionBasedCalculator>(position_based_calculator);


   visualizeReach( start_frame_reaching_isometry,
                  end_frame_reaching_isometry,
                  frame_aimed_at_isometry,
                  visual_tools_,
                  pbc2);

  Eigen::Vector3d j_plane_normal( start_frame_reaching.rotation()(0,2),
                                    start_frame_reaching.rotation()(1,2),
                                    start_frame_reaching.rotation()(2,2));

  double angle = fabrik::signedAngleBetweenTwoVectors(pbc2->start_to_end_projected, 
                                                      pbc2->start_to_aim_projected,
                                                      j_plane_normal);

  std::cout << "reaching angle: " << angle * 180 / M_PI << std::endl;

  // ------------------

  visual_tools_->trigger();
  visual_tools_->prompt("done");

return 0;
}


