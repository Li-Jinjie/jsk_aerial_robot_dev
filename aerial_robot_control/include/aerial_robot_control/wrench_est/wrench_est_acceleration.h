//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"

namespace aerial_robot_control
{

class WrenchEstAcceleration : public aerial_robot_control::WrenchEstActuatorMeasBase
{
public:
  WrenchEstAcceleration() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstActuatorMeasBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

    // acceleration-based method
    force_acc_alpha_matrix_ = Eigen::MatrixXd::Identity(3, 3);
    torque_acc_alpha_matrix_ = Eigen::MatrixXd::Identity(3, 3);
    double force_alpha_weight, torque_alpha_weight;
    ros::NodeHandle acceleration_nh(nh_, "controller/acceleration_observer");
    getParam<double>(acceleration_nh, "force_alpha_weight", force_alpha_weight, 0.5);
    getParam<double>(acceleration_nh, "torque_alpha_weight", torque_alpha_weight, 0.5);
    force_acc_alpha_matrix_ *= force_alpha_weight;
    torque_acc_alpha_matrix_ *= torque_alpha_weight;
  }

  void reset() override
  {
    WrenchEstActuatorMeasBase::reset();
    est_external_force_ = Eigen::VectorXd::Zero(3);
    est_external_torque_ = Eigen::VectorXd::Zero(3);
  }

  void update() override
  {
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));

    Eigen::VectorXd target_wrench_cog = calcWrenchFromActuatorMeas();
    Eigen::VectorXd target_force_cog = target_wrench_cog.head(3);
    Eigen::VectorXd target_torque_cog = target_wrench_cog.tail(3);

    // force estimation
    Eigen::Vector3d specific_force_cog;  // the specific force of CoG point in CoG frame, i.e., acceleration - gravity
    tf::vectorTFToEigen(imu_handler->getFilteredAccCogInCog(), specific_force_cog);

    double mass = robot_model_->getMass();

    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);

    auto est_external_force_now = cog_rot * (mass * specific_force_cog - target_force_cog);

    // low pass filter  TODO: try a better filter
    // TODO: put this part to the estimator to perform the same frequency with the IMU.
    est_external_force_ = (Eigen::MatrixXd::Identity(3, 3) - force_acc_alpha_matrix_) * est_external_force_ +
                          force_acc_alpha_matrix_ * est_external_force_now;

    setDistForceW(est_external_force_(0), est_external_force_(1), est_external_force_(2));

    // torque estimation
    Eigen::Vector3d omega_cog, omega_dot_cog;
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCogInCog(), omega_cog);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaDotCogInCog(), omega_dot_cog);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::VectorXd torque_imu_cog_ =
        inertia * omega_dot_cog + aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    auto external_torque_now = torque_imu_cog_ - target_torque_cog;

    // low pass filter  TODO: try a better filter
    est_external_torque_ = (Eigen::MatrixXd::Identity(3, 3) - torque_acc_alpha_matrix_) * est_external_torque_ +
                           torque_acc_alpha_matrix_ * external_torque_now;

    setDistTorqueCOG(est_external_torque_(0), est_external_torque_(1), est_external_torque_(2));
  }

private:
  // acceleration-based method
  Eigen::MatrixXd force_acc_alpha_matrix_;
  Eigen::VectorXd est_external_force_;

  Eigen::MatrixXd torque_acc_alpha_matrix_;
  Eigen::VectorXd est_external_torque_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
