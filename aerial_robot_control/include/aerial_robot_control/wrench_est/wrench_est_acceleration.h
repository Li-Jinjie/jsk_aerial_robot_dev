//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"
#include "spinal/FourAxisCommand.h"

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
    est_ext_force_cog_filtered_ = Eigen::VectorXd::Zero(3);
    est_ext_torque_cog_filtered_ = Eigen::VectorXd::Zero(3);
  }

  Eigen::VectorXd calDistWrench()
  {
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));

    Eigen::VectorXd target_wrench_cog = calcWrenchFromActuatorMeas();
    Eigen::VectorXd target_force_cog = target_wrench_cog.head(3);
    Eigen::VectorXd target_torque_cog = target_wrench_cog.tail(3);

    // force estimation
    Eigen::Vector3d specific_force_cog;  // the specific force of CoG point in CoG frame, i.e., acceleration - gravity
    tf::vectorTFToEigen(imu_handler->getFilteredAccCogInCog(), specific_force_cog);

    double mass = robot_model_->getMass();

    auto external_force_cog = mass * specific_force_cog - target_force_cog;

    // torque estimation
    Eigen::Vector3d omega_cog, omega_dot_cog;
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCogInCog(), omega_cog);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaDotCogInCog(), omega_dot_cog);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::VectorXd torque_imu_cog =
        inertia * omega_dot_cog + aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    auto external_torque_cog = torque_imu_cog - target_torque_cog;

    // combine the force and torque
    Eigen::VectorXd external_wrench_cog(6);
    external_wrench_cog << external_force_cog, external_torque_cog;
    return external_wrench_cog;
  }

  void update() override
  {
    /* calculate the external wrench in the CoG frame */
    Eigen::VectorXd external_wrench_cog = calDistWrench();
    Eigen::VectorXd external_force_cog = external_wrench_cog.head(3);
    Eigen::VectorXd external_torque_cog = external_wrench_cog.tail(3);

    /* force */
    // low pass filter  TODO: try a better filter
    // TODO: put this part to the estimator to perform the same frequency with the IMU.
    est_ext_force_cog_filtered_ =
        (Eigen::MatrixXd::Identity(3, 3) - force_acc_alpha_matrix_) * est_ext_force_cog_filtered_ +
        force_acc_alpha_matrix_ * external_force_cog;

    // coordinate transformation
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);
    auto est_ext_force_w_filtered = cog_rot * est_ext_force_cog_filtered_;

    setDistForceW(est_ext_force_w_filtered(0), est_ext_force_w_filtered(1), est_ext_force_w_filtered(2));

    /* torque */
    // low pass filter  TODO: try a better filter
    est_ext_torque_cog_filtered_ =
        (Eigen::MatrixXd::Identity(3, 3) - torque_acc_alpha_matrix_) * est_ext_torque_cog_filtered_ +
        torque_acc_alpha_matrix_ * external_torque_cog;

    setDistTorqueCOG(est_ext_torque_cog_filtered_(0), est_ext_torque_cog_filtered_(1), est_ext_torque_cog_filtered_(2));
  }

  void updateINDI(spinal::FourAxisCommand& flight_cmd_old, sensor_msgs::JointState& gimbal_ctrl_cmd_old,
                  spinal::FourAxisCommand& flight_cmd_new, sensor_msgs::JointState& gimbal_ctrl_cmd_new)  // TODO： separate it
  {
    Eigen::VectorXd external_wrench_cog = calDistWrench();

    Eigen::VectorXd dist_wrench_res_cog = Eigen::VectorXd::Zero(6);
    dist_wrench_res_cog.head(3) = external_wrench_cog.head(3) - est_ext_force_cog_filtered_;
    dist_wrench_res_cog.tail(3) = external_wrench_cog.tail(3) - est_ext_torque_cog_filtered_;

    Eigen::VectorXd d_z = alloc_mat_pinv_ * (-dist_wrench_res_cog);

    Eigen::VectorXd z_mpc = Eigen::VectorXd::Zero(2 * robot_model_->getRotorNum());
    for (int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      z_mpc(2 * i) = flight_cmd_old.base_thrust[i] * sin(gimbal_ctrl_cmd_old.position[i]);
      z_mpc(2 * i + 1) = flight_cmd_old.base_thrust[i] * cos(gimbal_ctrl_cmd_old.position[i]);
    }

    Eigen::VectorXd z = z_mpc + d_z;

    flight_cmd_new = flight_cmd_old;
    gimbal_ctrl_cmd_new = gimbal_ctrl_cmd_old;

    for (int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      flight_cmd_new.base_thrust[i] = (float)sqrt(z(2 * i) * z(2 * i) + z(2 * i + 1) * z(2 * i + 1));
      gimbal_ctrl_cmd_new.position[i] = atan2(z(2 * i), z(2 * i + 1));
    }
  }

private:
  // acceleration-based method
  Eigen::MatrixXd force_acc_alpha_matrix_;
  Eigen::VectorXd est_ext_force_cog_filtered_;  // TODO: combine these two variables and matrices to a single one

  Eigen::MatrixXd torque_acc_alpha_matrix_;
  Eigen::VectorXd est_ext_torque_cog_filtered_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_ACCELERATION_H
