#include "w10_kinematics/w10_kinematics_solver.hpp"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main() {
  try {
    // Get the path to w10.urdf
    std::string w10_sim_share = ament_index_cpp::get_package_share_directory("w10_sim");
    std::string urdf_path = w10_sim_share + "/urdf/w10.urdf";

    std::cout << "Loading URDF from: " << urdf_path << std::endl;

    // Create kinematics solver
    w10_kinematics::W10KinematicsSolver solver(urdf_path);

    // Test forward kinematics with a zero configuration
    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(solver.getNDOF());
    Eigen::Affine3d T;
    
    if (solver.forwardKinematics(q_zero, T)) {
      std::cout << "Forward kinematics successful" << std::endl;
      std::cout << "End-effector position: " << T.translation().transpose() << std::endl;
    } else {
      std::cerr << "Forward kinematics failed" << std::endl;
      return 1;
    }

    std::cout << "\nW10 Kinematics Test completed successfully" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
