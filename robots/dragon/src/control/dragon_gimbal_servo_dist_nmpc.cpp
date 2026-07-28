#include <dragon/control/dragon_gimbal_servo_dist_nmpc.h>

#include <aerial_robot_model/utils/math_utils.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerial_robot_control
{
void DragonGimbalServoDistNMPC::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_du)
{
  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::FullVectoringRobotModel>(robot_model);
  if (!dragon_robot_model_)
    throw std::runtime_error("DragonGimbalServoDistNMPC requires Dragon::FullVectoringRobotModel.");

  nmpc::TiltMtServoDistNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  if (motor_num_ != kModuleCount || joint_num_ != kGimbalCount)
    throw std::runtime_error("DRAGON NMPC requires four thrust modules and eight gimbal servos.");

  ROS_INFO("DRAGON gimbal-servo NMPC initialized at %.1f Hz.", ctrl_loop_du_);
}

void DragonGimbalServoDistNMPC::initAllocMat()
{
  const auto rotor_positions = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto link_rotations = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  if (rotor_positions.size() != kModuleCount || link_rotations.size() != kModuleCount)
    throw std::runtime_error("DRAGON robot model did not provide four module poses.");

  alloc_mat_ = Eigen::MatrixXd::Zero(6, 3 * kModuleCount);
  for (int module = 0; module < kModuleCount; ++module)
  {
    alloc_mat_.block<3, 3>(0, 3 * module) = link_rotations[module];
    alloc_mat_.block<3, 3>(3, 3 * module) = aerial_robot_model::skew(rotor_positions[module]) * link_rotations[module];
  }
  alloc_mat_pinv_ = aerial_robot_model::pseudoinverse(alloc_mat_);
  fix_rotor_idx_prev_ = -1;

  if (wrench_est_ptr_)
    wrench_est_ptr_->init_alloc_mtx(alloc_mat_, alloc_mat_pinv_);
}

std::vector<double> DragonGimbalServoDistNMPC::dragonPhysicalParams() const
{
  const auto rotor_positions = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto link_rotations = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  if (rotor_positions.size() != kModuleCount || link_rotations.size() != kModuleCount)
    throw std::runtime_error("DRAGON robot model did not provide four module poses.");

  std::vector<double> parameters;
  parameters.reserve(kPhysicalParameterCount);
  parameters.push_back(mass_);
  parameters.push_back(gravity_const_);
  parameters.insert(parameters.end(), inertia_.begin(), inertia_.end());

  for (int module = 0; module < kModuleCount; ++module)
  {
    parameters.push_back(rotor_positions[module].x());
    parameters.push_back(rotor_positions[module].y());
    parameters.push_back(rotor_positions[module].z());

    Eigen::Quaterniond quaternion(link_rotations[module]);
    quaternion.normalize();
    parameters.push_back(quaternion.w());
    parameters.push_back(quaternion.x());
    parameters.push_back(quaternion.y());
    parameters.push_back(quaternion.z());
  }

  parameters.push_back(t_rotor_);
  parameters.push_back(t_servo_);
  if (parameters.size() != kPhysicalParameterCount)
    throw std::runtime_error("Internal error while constructing DRAGON NMPC physical parameters.");
  return parameters;
}

void DragonGimbalServoDistNMPC::initNMPCParams()
{
  updateInertialParams();

  std::vector<double> parameters(mpc_solver_ptr_->NP_, 0.0);
  parameters[0] = 1.0;
  idx_p_quat_end_ = 3;

  auto physical_parameters = dragonPhysicalParams();
  std::copy(physical_parameters.begin(), physical_parameters.end(), parameters.begin() + 4);
  idx_p_phys_end_ = 4 + static_cast<int>(physical_parameters.size()) - 1;
  idx_p_dist_end_ = idx_p_phys_end_ + 6;

  if (idx_p_dist_end_ + 1 != mpc_solver_ptr_->NP_)
    throw std::runtime_error("DRAGON NMPC parameter dimension does not match the generated solver.");
  mpc_solver_ptr_->setParameters(parameters);
}

void DragonGimbalServoDistNMPC::prepareNMPCParams()
{
  updateInertialParams();
  auto physical_parameters = dragonPhysicalParams();
  mpc_solver_ptr_->setParameters(physical_parameters, idx_p_quat_end_ + 1);

  const auto model_error_force_world = wrench_est_i_term_.getDistForceW();
  const auto model_error_torque_cog = wrench_est_i_term_.getDistTorqueCOG();
  std::vector<double> disturbance_parameters{ model_error_force_world.x, model_error_force_world.y,
                                              model_error_force_world.z, model_error_torque_cog.x,
                                              model_error_torque_cog.y,  model_error_torque_cog.z };
  mpc_solver_ptr_->setParameters(disturbance_parameters, idx_p_phys_end_ + 1);
}

void DragonGimbalServoDistNMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                             const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                             const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                                             std::vector<double>& u)
{
  x.at(0) = ref_pos_i.x();
  x.at(1) = ref_pos_i.y();
  x.at(2) = ref_pos_i.z();
  x.at(3) = ref_vel_i.x();
  x.at(4) = ref_vel_i.y();
  x.at(5) = ref_vel_i.z();
  x.at(6) = ref_quat_ib.w();
  x.at(7) = ref_quat_ib.x();
  x.at(8) = ref_quat_ib.y();
  x.at(9) = ref_quat_ib.z();
  x.at(10) = ref_omega_b.x();
  x.at(11) = ref_omega_b.y();
  x.at(12) = ref_omega_b.z();

  const Eigen::VectorXd module_forces = alloc_mat_pinv_ * ref_wrench_b;
  for (int module = 0; module < kModuleCount; ++module)
  {
    const Eigen::Vector3d force_link = module_forces.segment<3>(3 * module);
    const double thrust = force_link.norm();
    double roll = 0.0;
    double pitch = 0.0;
    if (thrust > 1.0e-8)
    {
      roll = std::atan2(-force_link.y(), force_link.z());
      const double signed_pitch_denominator = -force_link.y() * std::sin(roll) + force_link.z() * std::cos(roll);
      pitch = std::atan2(force_link.x(), signed_pitch_denominator);
    }

    const int roll_index = 2 * module;
    const int pitch_index = roll_index + 1;
    x.at(13 + roll_index) = ensureOneServoContinuity(roll, roll_index);
    x.at(13 + pitch_index) = ensureOneServoContinuity(pitch, pitch_index);
    u.at(module) = thrust;

    // The nonlinear least-squares input residual is a_command - a_servo.
    // Therefore the angle portion of its y-reference must remain zero.
    u.at(kModuleCount + roll_index) = 0.0;
    u.at(kModuleCount + pitch_index) = 0.0;
  }
}

void DragonGimbalServoDistNMPC::callbackJointStates(const sensor_msgs::JointStateConstPtr& msg)
{
  int matched = 0;
  for (int gimbal = 0; gimbal < kGimbalCount; ++gimbal)
  {
    const auto name_it = std::find(msg->name.begin(), msg->name.end(), gimbal_joint_names_[gimbal]);
    if (name_it == msg->name.end())
      continue;

    const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), name_it));
    if (index < msg->position.size())
    {
      joint_angles_[gimbal] = msg->position[index];
      ++matched;
    }
  }
  if (matched != kGimbalCount)
    ROS_WARN_THROTTLE(2.0, "DRAGON joint_states contains %d/8 named gimbal joints.", matched);
}

std::string DragonGimbalServoDistNMPC::getServoJointName(int index) const
{
  return gimbal_joint_names_.at(index);
}

}  // namespace aerial_robot_control

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonGimbalServoDistNMPC, aerial_robot_control::ControlBase)
