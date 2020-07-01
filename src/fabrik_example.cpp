#include <ros/ros.h>
#include <ros/console.h>
#include <fabrik/util/math.h>

#include <vector>

#include <fabrik/base/fabrik.h>

class MakerPlanar
{
public:

    MakerPlanar()
    {
        makeChain();
        makeBase();
    }

    void makeChain()
    {
        Eigen::Vector3d vec1(0,0,1);
        vec1.normalize();
        Eigen::Affine3d link1_frame(Eigen::AngleAxisd(0, vec1));
        link1_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link1("link1",  link1_frame);

        Eigen::Vector3d vec2(0,0,1);
        vec2.normalize();
        Eigen::Affine3d link2_frame(Eigen::AngleAxisd(0, vec2));
        link2_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link2("link2",  link2_frame);

        Eigen::Vector3d vec3(0,0,1);
        vec3.normalize();
        Eigen::Affine3d link3_frame(Eigen::AngleAxisd(0, vec3));
        link3_frame.translation() = Eigen::Vector3d(1, 0, 0);
        robot_model::Link link3("link3",  link3_frame);

        chain.push_back(link1);
        chain.push_back(link2);
        chain.push_back(link3);
    }

    void makeBase()
    {
        Eigen::Vector3d vec0(0,0,1);
        vec0.normalize();
        Eigen::Affine3d base_transformation(Eigen::AngleAxisd(0, vec0));
        base_transformation.translate(Eigen::Vector3d(0, 0, 0));
        base = base_transformation;
    }

    std::vector<robot_model::Link> chain;
    Eigen::Affine3d base;
};


int main(int argc, char** argv)
{
  const std::string NODE_NAME = "fabrik_example";
  ros::init(argc, argv, NODE_NAME);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  std::cout << "--------- test " << std::endl;

  double randm_num = fabrik::randomDouble(0, 1);
  std::cout << "random num from fabrik: " << randm_num << std::endl;

  MakerPlanar maker;
  std::vector<double> fk_joints_values_1 = {M_PI_4, 0, 0};

  // ---------------------- Solve another forward kinematics close to the first one:
    robot_state::RobotStatePtr robot_state_2 = 
        std::make_shared<robot_state::RobotState>(maker.chain, maker.base);
    
    robot_state_2->setReachingDirection(robot_state::ReachingDirection::FORWARD);
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

    fabrik::FABRIKPtr fabrik(new fabrik::FABRIK(maker.base,
                                                maker.chain,
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
}