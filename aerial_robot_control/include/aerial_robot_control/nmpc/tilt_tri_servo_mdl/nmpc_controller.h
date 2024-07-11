//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_TRI_NMPC_CONTROLLER_H
#define TILT_TRI_NMPC_CONTROLLER_H

#endif  // TILT_TRI_NMPC_CONTROLLER_H

#pragma once

#include "aerial_robot_control/control/base/base.h"
#include "nmpc_solver.h"

#include <angles/angles.h>
#include <tf_conversions/tf_eigen.h>

/* dynamic reconfigure */
#include "aerial_robot_msgs/DynamicReconfigureLevels.h"
#include "aerial_robot_control/NMPCConfig.h"
#include <dynamic_reconfigure/server.h>

/* protocol */
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "aerial_robot_msgs/PredXU.h"
#include "spinal/FourAxisCommand.h"
#include "spinal/RollPitchYawTerms.h"
#include "spinal/PMatrixPseudoInverseWithInertia.h"
#include "spinal/SetControlMode.h"
#include "spinal/DesireCoord.h"

/* action */
#include "actionlib/server/simple_action_server.h"
#include "aerial_robot_msgs/PredXU.h"
#include "aerial_robot_msgs/TrackTrajAction.h"
#include "aerial_robot_msgs/TrackTrajFeedback.h"
#include "aerial_robot_msgs/TrackTrajGoal.h"
#include "aerial_robot_msgs/TrackTrajResult.h"

using NMPCControlDynamicConfig = dynamic_reconfigure::Server<aerial_robot_control::NMPCConfig>;

namespace aerial_robot_control
{
namespace nmpc_tilt_tri_full
{

class NMPCController : public ControlBase
{
public:
  NMPCController() = default;  // note that constructor should not have arguments as the rule of rospluginlib
  ~NMPCController() override = default;
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  bool update() override;
  void reset() override;

protected:
  ros::Timer tmr_viz_;

  ros::Publisher pub_flight_cmd_;                       // for spinal
  ros::Publisher pub_viz_pred_;                         // for viz predictions
  ros::Publisher pub_viz_ref_;                          // for viz reference
  ros::Publisher pub_rpy_gain_;                         // for gains of attitude controller
  ros::Publisher pub_p_matrix_pseudo_inverse_inertia_;  // for pseudo inverse inertia
  ros::Publisher pub_gimbal_control_;                   // for gimbal control

  ros::ServiceClient srv_set_control_mode_;
  std::vector<boost::shared_ptr<NMPCControlDynamicConfig> > nmpc_reconf_servers_;

  ros::Subscriber sub_joint_states_;
  ros::Subscriber sub_set_rpy_;
  ros::Subscriber sub_set_ref_traj_;

  bool is_attitude_ctrl_;
  bool is_body_rate_ctrl_;
  bool is_print_phys_params_;
  bool is_debug_;

  virtual void controlCore();
  virtual void SendCmd();

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level);

private:
  double mass_;
  double gravity_const_;
  double t_nmpc_samp_;
  double t_nmpc_integ_;

  double joint_angles_[3] = { 0.0, 0.0, 0.0 };

  bool is_init_alloc_mat_ = false;  // TODO: tmp value. should be combined with KDL framework in the future
  Eigen::Matrix<double, 6, 6> alloc_mat_;
  Eigen::MatrixXd alloc_mat_pinv_;

  bool is_traj_tracking_ = false;  // TODO: tmp value. should be combined with inner traj. tracking in the future
  ros::Time receive_time_;         // tmp value. should be combined with inner traj. tracking in the future

  nav_msgs::Odometry odom_;
  aerial_robot_msgs::PredXU x_u_ref_;
  spinal::FourAxisCommand flight_cmd_;
  sensor_msgs::JointState gimbal_ctrl_cmd_;

  MPCSolver mpc_solver_;

  nav_msgs::Odometry getOdom();
  void callbackViz(const ros::TimerEvent& event);
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg);
  void callbackSetRPY(const spinal::DesireCoordConstPtr& msg);
  void callbackSetRefTraj(const aerial_robot_msgs::PredXUConstPtr& msg);

  void initAllocMat();
  void calXrUrRef(const tf::Vector3 target_pos, const tf::Vector3 target_vel, const tf::Vector3 target_rpy,
                  const tf::Vector3 target_omega, const Eigen::VectorXd& target_wrench);

  double getCommand(int idx_u, double t_pred = 0.0);

  void printPhysicalParams();
};

};  // namespace nmpc_tilt_tri_full

};  // namespace aerial_robot_control