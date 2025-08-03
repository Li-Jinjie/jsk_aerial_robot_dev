//
// Created by li-jinjie on 24-10-25.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H

#include "aerial_robot_control/wrench_est/wrench_est_actuator_meas_base.h"

namespace aerial_robot_control
{

class WrenchEstMomentum : public aerial_robot_control::WrenchEstActuatorMeasBase
{
public:
  WrenchEstMomentum() = default;

  void initialize(ros::NodeHandle& nh, boost::shared_ptr<aerial_robot_model::RobotModel>& robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator>& estimator, double ctrl_loop_du) override
  {
    WrenchEstActuatorMeasBase::initialize(nh, robot_model, estimator, ctrl_loop_du);

    // initialize the matrix
    momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6, 6);
    double force_weight, torque_weight;
    ros::NodeHandle momentum_nh(nh_, "controller/momentum_observer");
    getParam<double>(momentum_nh, "momentum_observer_force_weight", force_weight, 3.0);
    getParam<double>(momentum_nh, "momentum_observer_torque_weight", torque_weight, 2.0);
    momentum_observer_matrix_.topRows(3) *= force_weight;
    momentum_observer_matrix_.bottomRows(3) *= torque_weight;
  }

  void reset() override
  {
    WrenchEstActuatorMeasBase::reset();
    est_external_wrench_ = Eigen::VectorXd::Zero(6);
    init_sum_momentum_ = Eigen::VectorXd::Zero(6);
    integrate_term_ = Eigen::VectorXd::Zero(6);
    prev_est_wrench_timestamp_ = 0;
  }

  void update() override
  {
    Eigen::Vector3d vel_w, omega_cog;
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu4WrenchEst>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getVelCogInW(), vel_w);        // the vel of CoG point in world frame
    tf::vectorTFToEigen(imu_handler->getOmegaCogInCog(), omega_cog);  // the omega of CoG point in CoG frame
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimator_->getEstimateMode()), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6, 6);
    J_t.topLeftCorner(3, 3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();                    // mg
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);  // omega x (I omega)

    Eigen::VectorXd target_wrench_cog = calcWrenchFromActuatorMeas(thrust_meas_, joint_angles_);

    if (prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum;  // not good
    }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

    // for est_external_wrench_, force is in world frame, torque is in CoG frame
    est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();

    /* set value */
    setDistForceW(est_external_wrench_(0), est_external_wrench_(1), est_external_wrench_(2));
    setDistTorqueCOG(est_external_wrench_(3), est_external_wrench_(4), est_external_wrench_(5));

    WrenchEstActuatorMeasBase::update();
  }

private:
  Eigen::VectorXd init_sum_momentum_;
  Eigen::VectorXd est_external_wrench_;
  Eigen::MatrixXd momentum_observer_matrix_;
  Eigen::VectorXd integrate_term_;
  double prev_est_wrench_timestamp_;
};

};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_MOMENTUM_H
