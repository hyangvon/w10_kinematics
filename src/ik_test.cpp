#include "w10_kinematics/arm_angle_ik.hpp"
#include <iostream>
#include <iomanip>

void printConfig(const Eigen::VectorXd& q, const std::string& label) {
  std::cout << label << ": ";
  for (int i = 0; i < q.size(); ++i) {
    std::cout << std::fixed << std::setprecision(4) << q(i) << " ";
  }
  std::cout << std::endl;
}

int main() {
  try {
    std::string urdf_path = "/home/user/ros2_ws/dynamic_ws/install/w10_sim/share/w10_sim/urdf/w10.urdf";

    std::cout << "========================================" << std::endl;
    std::cout << "   W10 Arm Angle Dimension Reduction IK" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "URDF: " << urdf_path << std::endl << std::endl;

    w10_kinematics::ArmAngleIK ik_solver(urdf_path);

    int ndof = ik_solver.getNDOF();
    std::cout << "[Test] Total DOF: " << ndof << std::endl;
    std::cout << "[Test] Total Bodies: " << ik_solver.getNBodies() << std::endl << std::endl;

    std::cout << "--- Test 1: Forward Kinematics (Zero Config) ---" << std::endl;
    Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(ndof);
    Eigen::Isometry3d T_zero;

    if (ik_solver.forwardKinematics(q_zero, T_zero)) {
      printConfig(q_zero, "Config");
      std::cout << "End-Effector Position: " << T_zero.translation().transpose() << std::endl;
      double arm_angle = ik_solver.getArmAngle(q_zero);
      std::cout << "Arm Angle: " << arm_angle << " rad (" << (arm_angle * 180 / M_PI) << " deg)" << std::endl;
    } else {
      std::cerr << "FK failed" << std::endl;
      return 1;
    }

    std::cout << "\n--- Test 2: Forward Kinematics (Custom Config) ---" << std::endl;
    Eigen::VectorXd q_custom = Eigen::VectorXd::Zero(ndof);
    q_custom(2) = 0.5;
    q_custom(3) = 0.3;
    q_custom(4) = -0.8;
    Eigen::Isometry3d T_custom;

    if (ik_solver.forwardKinematics(q_custom, T_custom)) {
      printConfig(q_custom, "Config");
      std::cout << "End-Effector Position: " << T_custom.translation().transpose() << std::endl;
      double arm_angle = ik_solver.getArmAngle(q_custom);
      std::cout << "Arm Angle: " << arm_angle << " rad (" << (arm_angle * 180 / M_PI) << " deg)" << std::endl;
    }

    std::cout << "\n--- Test 3: Arm Angle Calculation ---" << std::endl;
    for (int i = 0; i < 3; ++i) {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(ndof);
      q(4) = -0.5 + i * 0.5;
      double aa = ik_solver.getArmAngle(q);
      std::cout << "  joint5=" << std::fixed << std::setprecision(2) << q(4) 
                << " -> Arm Angle=" << aa << " rad" << std::endl;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "        All Tests Completed Successfully" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
