#include "rclcpp/rclcpp.hpp"
#include "w10_kinematics/w10_kinematics_solver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("w10_ik_node");

  // Get the path to w10.urdf
  std::string w10_sim_share = ament_index_cpp::get_package_share_directory("w10_sim");
  std::string urdf_path = w10_sim_share + "/urdf/w10.urdf";

  // Create kinematics solver
  w10_kinematics::W10KinematicsSolver solver(urdf_path);

  RCLCPP_INFO(node->get_logger(), "W10 Inverse Kinematics Node initialized");
  RCLCPP_INFO(node->get_logger(), "Robot DOF: %d", solver.getNDOF());

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
