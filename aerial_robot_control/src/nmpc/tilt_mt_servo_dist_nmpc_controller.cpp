//
// Created by lijinjie on 24/07/18.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  TiltMtServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "if_use_est_wrench_4_control", if_use_est_wrench_4_control_, false);

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("ext_wrench_est/value", 1);
}

void nmpc::TiltMtServoDistNMPC::controlCore()
{
  updateITerm();

  updateDisturbWrench();  // should be called before controlCore of parent class to keep fresh

  TiltMtServoNMPC::controlCore();
}

void nmpc::TiltMtServoDistNMPC::sendCmd()
{
  TiltMtServoNMPC::sendCmd();

  pubDisturbWrench();  // should be called after sendCmd of parent class to keep control input fresh
}

void nmpc::TiltMtServoDistNMPC::resetPlugins()
{
  wrench_est_i_term_.reset();

  if (wrench_est_ptr_ != nullptr)
    wrench_est_ptr_->reset();
}

void nmpc::TiltMtServoDistNMPC::initPlugins()
{
  /* I Term is always loaded  */
  wrench_est_i_term_.initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);

  /* plugin: wrench estimator */
  wrench_est_loader_ptr_ = boost::make_shared<pluginlib::ClassLoader<aerial_robot_control::WrenchEstActuatorMeasBase>>(
      "aerial_robot_control", "aerial_robot_control::WrenchEstActuatorMeasBase");
  try
  {
    // 1. read the plugin name from the parameter server
    std::string wrench_estimator_name;
    nh_.param("wrench_estimator_name", wrench_estimator_name, std::string("aerial_robot_control::WrenchEstNone"));

    // 2. load the plugin
    wrench_est_ptr_ = wrench_est_loader_ptr_->createInstance(wrench_estimator_name);
    wrench_est_ptr_->initialize(nh_, robot_model_, estimator_, ctrl_loop_du_);
    ROS_INFO("load wrench estimator plugin: %s", wrench_estimator_name.c_str());
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("wrench_est_plugin: The plugin failed to load for some reason. Error: %s", ex.what());
  }
}

void nmpc::TiltMtServoDistNMPC::updateITerm()
{
  /* HANDLING MODEL ERROR */
  /* get the current state */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Quaternion q = estimator_->getQuat(Frame::COG, estimate_mode_);

  /* get the target state */
  // Note that the target state is the estimated state from NMPC solver.
  tf::Vector3 pos_x0(mpc_solver_ptr_->xo_.at(0).at(0), mpc_solver_ptr_->xo_.at(0).at(1),
                     mpc_solver_ptr_->xo_.at(0).at(2));
  tf::Vector3 pos_x1(mpc_solver_ptr_->xo_.at(1).at(0), mpc_solver_ptr_->xo_.at(1).at(1),
                     mpc_solver_ptr_->xo_.at(1).at(2));
  tf::Vector3 target_pos = pos_x0 + (pos_x1 - pos_x0) * t_nmpc_samp_ / t_nmpc_step_;

  tf::Quaternion quat_x0(mpc_solver_ptr_->xo_.at(0).at(7), mpc_solver_ptr_->xo_.at(0).at(8),
                         mpc_solver_ptr_->xo_.at(0).at(9), mpc_solver_ptr_->xo_.at(0).at(6));
  quat_x0.normalize();
  tf::Quaternion quat_x1(mpc_solver_ptr_->xo_.at(1).at(7), mpc_solver_ptr_->xo_.at(1).at(8),
                         mpc_solver_ptr_->xo_.at(1).at(9), mpc_solver_ptr_->xo_.at(1).at(6));
  quat_x1.normalize();
  tf::Quaternion target_q = quat_x0.slerp(quat_x1, t_nmpc_samp_ / t_nmpc_step_);

  /* update I term */
  wrench_est_i_term_.update(target_pos, target_q, pos, q);
}

void nmpc::TiltMtServoDistNMPC::initNMPCParams()
{
  TiltMtServoNMPC::initNMPCParams();
  idx_p_dist_end_ = idx_p_phys_end_ + 6;
}

void nmpc::TiltMtServoDistNMPC::prepareNMPCParams()
{
  TiltMtServoNMPC::prepareNMPCParams();

  auto mdl_error_force_w = wrench_est_i_term_.getDistForceW();
  auto mdl_error_torque_cog = wrench_est_i_term_.getDistTorqueCOG();

  vector<double> p = { mdl_error_force_w.x,    mdl_error_force_w.y,    mdl_error_force_w.z,
                       mdl_error_torque_cog.x, mdl_error_torque_cog.y, mdl_error_torque_cog.z };
  mpc_solver_ptr_->setParameters(p, idx_p_phys_end_ + 1);
}

std::vector<double> nmpc::TiltMtServoDistNMPC::meas2VecX(bool is_modified_by_traj_frame)
{
  vector<double> bx0 = TiltMtServoNMPC::meas2VecX(is_modified_by_traj_frame);

  /* disturbance rejection */
  geometry_msgs::Vector3 external_force_w;     // default: 0, 0, 0
  geometry_msgs::Vector3 external_torque_cog;  // default: 0, 0, 0

  if (if_use_est_wrench_4_control_)
  {
    external_force_w = wrench_est_ptr_->getDistForceW();
    external_torque_cog = wrench_est_ptr_->getDistTorqueCOG();
  }

  bx0[13 + joint_num_ + 0] = external_force_w.x;
  bx0[13 + joint_num_ + 1] = external_force_w.y;
  bx0[13 + joint_num_ + 2] = external_force_w.z;
  bx0[13 + joint_num_ + 3] = external_torque_cog.x;
  bx0[13 + joint_num_ + 4] = external_torque_cog.y;
  bx0[13 + joint_num_ + 5] = external_torque_cog.z;

  return bx0;
}

void nmpc::TiltMtServoDistNMPC::updateDisturbWrench() const
{
  if (wrench_est_ptr_ == nullptr)
  {
    ROS_ERROR("wrench_est_ptr_ is nullptr, please check the plugin loading.");
    return;
  }

  auto vel = estimator_->getVel(Frame::COG, estimate_mode_);
  auto ang_vel = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  wrench_est_ptr_->update(vel, ang_vel);
}

void nmpc::TiltMtServoDistNMPC::pubDisturbWrench() const
{
  geometry_msgs::WrenchStamped dist_wrench_;
  dist_wrench_.header.frame_id = nh_.getNamespace().substr(1) + "/cog";

  auto ext_force_w = wrench_est_ptr_->getDistForceW();
  dist_wrench_.wrench.torque = wrench_est_ptr_->getDistTorqueCOG();

  tf::Matrix3x3 rot_mtx_cog2w = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 tf_dist_force_w = tf::Vector3(ext_force_w.x, ext_force_w.y, ext_force_w.z);
  tf::Vector3 tf_dist_force_cog = rot_mtx_cog2w.inverse() * tf_dist_force_w;
  dist_wrench_.wrench.force.x = tf_dist_force_cog.x();
  dist_wrench_.wrench.force.y = tf_dist_force_cog.y();
  dist_wrench_.wrench.force.z = tf_dist_force_cog.z();

  dist_wrench_.header.stamp = ros::Time::now();
  pub_disturb_wrench_.publish(dist_wrench_);
}

void nmpc::TiltMtServoDistNMPC::initAllocMat()
{
  TiltMtServoNMPC::initAllocMat();

  if (wrench_est_ptr_ != nullptr)
    wrench_est_ptr_->init_alloc_mtx(alloc_mat_, alloc_mat_pinv_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoDistNMPC, aerial_robot_control::ControlBase);
