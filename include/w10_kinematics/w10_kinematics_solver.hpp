#ifndef W10_KINEMATICS_SOLVER_HPP_
#define W10_KINEMATICS_SOLVER_HPP_

#include <Eigen/Dense>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <memory>

namespace w10_kinematics {

class W10KinematicsSolver {
public:
  using VectorXd = Eigen::VectorXd;
  using MatrixXd = Eigen::MatrixXd;

  W10KinematicsSolver(const std::string& urdf_path);
  ~W10KinematicsSolver() = default;

  // Forward kinematics
  bool forwardKinematics(const VectorXd& q, Eigen::Affine3d& T);

  // Inverse kinematics using arm angle dimension reduction
  bool inverseKinematics(
    const Eigen::Affine3d& T_target,
    const VectorXd& q_init,
    VectorXd& q_solution,
    double tolerance = 1e-6,
    int max_iterations = 1000);

  // Get robot model and data
  const pinocchio::Model& getModel() const { return model_; }
  pinocchio::Data& getData() { return data_; }

  // Get DOF count
  int getNDOF() const { return model_.nq; }

private:
  pinocchio::Model model_;
  pinocchio::Data data_;

  // Helpers for dimension reduction
  VectorXd reduceArmAngle(const VectorXd& q_full);
  VectorXd expandArmAngle(const VectorXd& q_reduced);
};

}  // namespace w10_kinematics

#endif  // W10_KINEMATICS_SOLVER_HPP_
