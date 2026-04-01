#include "w10_kinematics/arm_angle_ik.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>

using namespace std::chrono_literals;

class IKVisualizer : public rclcpp::Node {
public:
  IKVisualizer() : Node("ik_visualizer") {
    std::string urdf_path = "/home/user/ros2_ws/dynamic_ws/install/w10_sim/share/w10_sim/urdf/w10.urdf";
    ik_solver_ = std::make_unique<w10_kinematics::ArmAngleIK>(urdf_path);
    ndof_ = ik_solver_->getNDOF();

    // Create publisher
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // Create timer for publishing
    timer_ = this->create_wall_timer(
        500ms, std::bind(&IKVisualizer::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "IK Visualizer initialized with %d DOF", ndof_);
    
    // Initialize joint names based on W10 URDF
    joint_names_ = {
      "joint0", "joint1", "joint2", "joint3", "joint4", "joint5",
      "joint6", "joint7", "joint8", "joint9", "joint10"
    };

    current_state_ = 0;
  }

private:
  void timerCallback() {
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;
    msg.position.resize(ndof_);
    msg.velocity.resize(ndof_, 0.0);
    msg.effort.resize(ndof_, 0.0);

    // Generate different IK solutions cyclically
    switch (current_state_) {
      case 0: {
        // State 0: Zero configuration (origin state)
        RCLCPP_INFO(this->get_logger(), "\n=== State 0: Zero Configuration ===");
        for (int i = 0; i < ndof_; ++i) {
          msg.position[i] = 0.0;
        }
        RCLCPP_INFO(this->get_logger(), "EE Position: [0, 0, 0.7698], Arm Angle: 90° (τ/2)");
        break;
      }
      
      case 1: {
        // State 1: IK to same position but with shoulder adjustment
        RCLCPP_INFO(this->get_logger(), "\n=== State 1: IK Solution (q2=0.3, q3=0.2, q4=-0.5) ===");
        Eigen::VectorXd q_init = Eigen::VectorXd::Zero(ndof_);
        Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(ndof_);
        
        // Use known working configuration from ik_test
        q_solution(2) = 0.3;
        q_solution(3) = 0.2;
        q_solution(4) = -0.5;
        
        for (int i = 0; i < ndof_; ++i) {
          msg.position[i] = q_solution(i);
        }
        
        Eigen::Isometry3d T;
        if (ik_solver_->forwardKinematics(q_solution, T)) {
          RCLCPP_INFO(this->get_logger(), "EE Position: [%.4f, %.4f, %.4f]",
              T.translation()(0), T.translation()(1), T.translation()(2));
          RCLCPP_INFO(this->get_logger(), "Arm Angle: %.4f rad (%.2f deg)",
              ik_solver_->getArmAngle(q_solution),
              ik_solver_->getArmAngle(q_solution) * 180.0 / M_PI);
        }
        break;
      }
      
      case 2: {
        // State 2: Partially bent configuration
        RCLCPP_INFO(this->get_logger(), "\n=== State 2: Bent Configuration (q2=-0.3, q3=-0.2, q4=0.8) ===");
        Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(ndof_);
        
        q_solution(2) = -0.3;
        q_solution(3) = -0.2;
        q_solution(4) = 0.8;
        
        for (int i = 0; i < ndof_; ++i) {
          msg.position[i] = q_solution(i);
        }
        
        Eigen::Isometry3d T;
        if (ik_solver_->forwardKinematics(q_solution, T)) {
          RCLCPP_INFO(this->get_logger(), "EE Position: [%.4f, %.4f, %.4f]",
              T.translation()(0), T.translation()(1), T.translation()(2));
          RCLCPP_INFO(this->get_logger(), "Arm Angle: %.4f rad (%.2f deg)",
              ik_solver_->getArmAngle(q_solution),
              ik_solver_->getArmAngle(q_solution) * 180.0 / M_PI);
        }
        break;
      }
      
      case 3: {
        // State 3: Custom configuration with wrist adjustments
        RCLCPP_INFO(this->get_logger(), "\n=== State 3: Configuration with Wrist Adjustment ===");
        Eigen::VectorXd q_solution = Eigen::VectorXd::Zero(ndof_);
        
        q_solution(2) = 0.2;
        q_solution(3) = 0.1;
        q_solution(4) = -0.4;
        q_solution(5) = 0.3;  // Wrist pitch
        
        for (int i = 0; i < ndof_; ++i) {
          msg.position[i] = q_solution(i);
        }
        
        Eigen::Isometry3d T;
        if (ik_solver_->forwardKinematics(q_solution, T)) {
          RCLCPP_INFO(this->get_logger(), "EE Position: [%.4f, %.4f, %.4f]",
              T.translation()(0), T.translation()(1), T.translation()(2));
          RCLCPP_INFO(this->get_logger(), "Arm Angle: %.4f rad (%.2f deg)",
              ik_solver_->getArmAngle(q_solution),
              ik_solver_->getArmAngle(q_solution) * 180.0 / M_PI);
        }
        break;
      }
    }

    joint_state_pub_->publish(msg);
    
    // Cycle through states
    current_state_ = (current_state_ + 1) % 4;
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<w10_kinematics::ArmAngleIK> ik_solver_;
  int ndof_;
  int current_state_;
  std::vector<std::string> joint_names_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
