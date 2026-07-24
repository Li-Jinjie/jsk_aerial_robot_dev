#include "aerial_robot_control/nmpc/tilt_mt_servo_force_imp_nmpc_controller.h"

using namespace aerial_robot_control;

namespace
{
using ForceImpSolver = mpc_solver::TiltQdServoDistCoGForceImpMdlMPCSolver;
}

void nmpc::TiltMtServoForceImpNMPC::initNMPCCostW()
{
  ros::NodeHandle nmpc_nh(nh_, "controller/nmpc");

  double enlarge_factor, pMxy, pMz, Qv_xy, Qv_z, Qp_xy, Qp_z;
  double Qw_xy, Qw_z, Qq_xy, Qq_z, Qa, Rt, Rac_d;
  getParam<double>(nmpc_nh, "enlarge_factor", enlarge_factor, 1.0);
  getParam<double>(nmpc_nh, "pMxy", pMxy, 1.5);
  getParam<double>(nmpc_nh, "pMz", pMz, 1.5);
  getParam<double>(nmpc_nh, "Qv_xy", Qv_xy, 10.0);
  getParam<double>(nmpc_nh, "Qv_z", Qv_z, 10.0);
  getParam<double>(nmpc_nh, "Qp_xy", Qp_xy, 6.0);
  getParam<double>(nmpc_nh, "Qp_z", Qp_z, 6.0);
  getParam<double>(nmpc_nh, "Qw_xy", Qw_xy, 5.0);
  getParam<double>(nmpc_nh, "Qw_z", Qw_z, 5.0);
  getParam<double>(nmpc_nh, "Qq_xy", Qq_xy, 300.0);
  getParam<double>(nmpc_nh, "Qq_z", Qq_z, 600.0);
  getParam<double>(nmpc_nh, "Qa", Qa, 1.0);
  getParam<double>(nmpc_nh, "Rt", Rt, 1.0);
  getParam<double>(nmpc_nh, "Rac_d", Rac_d, 250.0);

  const auto solver = boost::dynamic_pointer_cast<ForceImpSolver>(mpc_solver_ptr_);
  if (!solver)
    throw std::runtime_error("The MPC solver is not the CoG force-impedance model.");

  solver->setForceImpedanceWeight("pMx", pMxy, false);
  solver->setForceImpedanceWeight("pMy", pMxy, false);
  solver->setForceImpedanceWeight("pMz", pMz, false);
  solver->setForceImpedanceWeight("pDx", Qv_xy, false);
  solver->setForceImpedanceWeight("pDy", Qv_xy, false);
  solver->setForceImpedanceWeight("pDz", Qv_z, false);
  solver->setForceImpedanceWeight("pKx", Qp_xy, false);
  solver->setForceImpedanceWeight("pKy", Qp_xy, false);
  solver->setForceImpedanceWeight("pKz", Qp_z, false);
  solver->setEnlargeFactor(enlarge_factor);

  mpc_solver_ptr_->setCostWDiagElement(6, 0.0);
  mpc_solver_ptr_->setCostWDiagElement(7, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(8, Qq_xy);
  mpc_solver_ptr_->setCostWDiagElement(9, Qq_z);
  mpc_solver_ptr_->setCostWDiagElement(10, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(11, Qw_xy);
  mpc_solver_ptr_->setCostWDiagElement(12, Qw_z);
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rt, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);
}

void nmpc::TiltMtServoForceImpNMPC::setForceImpedanceParams()
{
  const auto solver = boost::dynamic_pointer_cast<ForceImpSolver>(mpc_solver_ptr_);
  if (!solver)
    throw std::runtime_error("The MPC solver is not the CoG force-impedance model.");

  const double* virtual_mass = solver->getVirtualMass();
  std::vector<double> parameters = { virtual_mass[0], virtual_mass[1], virtual_mass[2] };
  mpc_solver_ptr_->setParameters(parameters, idx_p_dist_end_ + 1);
}

void nmpc::TiltMtServoForceImpNMPC::initNMPCParams()
{
  TiltMtServoDistNMPC::initNMPCParams();
  setForceImpedanceParams();
}

void nmpc::TiltMtServoForceImpNMPC::updateForceAndLeverArmTorque()
{
  estimated_force_w_.setValue(0.0, 0.0, 0.0);
  lever_arm_torque_b_.setValue(0.0, 0.0, 0.0);

  if (!wrench_est_ptr_)
  {
    ROS_ERROR_THROTTLE(1.0, "The wrench estimator is unavailable; force impedance uses zero disturbance.");
    return;
  }

  const geometry_msgs::Vector3 force_w = wrench_est_ptr_->getDistForceW();
  estimated_force_w_.setValue(force_w.x, force_w.y, force_w.z);

  if (!robot_model_->hasFrame("ee_contact"))
  {
    ROS_WARN_THROTTLE(5.0, "No ee_contact frame; force-impedance lever-arm torque is set to zero.");
    return;
  }

  std::vector<double> ee_position_b;
  std::vector<double> ee_quaternion_b;
  robot_model_->getCoGtoFramePosQuat("ee_contact", ee_position_b, ee_quaternion_b);
  const tf::Vector3 lever_arm_b(ee_position_b[0], ee_position_b[1], ee_position_b[2]);

  const tf::Matrix3x3 rotation_wb = estimator_->getOrientation(Frame::COG, estimate_mode_);
  const tf::Vector3 force_b = rotation_wb.inverse() * estimated_force_w_;
  lever_arm_torque_b_ = lever_arm_b.cross(force_b);
}

void nmpc::TiltMtServoForceImpNMPC::controlCore(bool is_warmup)
{
  if (!is_warmup)
  {
    updateITerm();
    updateDisturbWrench();
    updateForceAndLeverArmTorque();
  }
  else
  {
    estimated_force_w_.setValue(0.0, 0.0, 0.0);
    lever_arm_torque_b_.setValue(0.0, 0.0, 0.0);
  }

  TiltMtServoNMPC::controlCore(is_warmup);
}

std::vector<double> nmpc::TiltMtServoForceImpNMPC::meas2VecX(bool is_modified_by_traj_frame)
{
  std::vector<double> state = TiltMtServoNMPC::meas2VecX(is_modified_by_traj_frame);
  const int disturbance_start = 13 + joint_num_;
  state.at(disturbance_start + 0) = estimated_force_w_.x();
  state.at(disturbance_start + 1) = estimated_force_w_.y();
  state.at(disturbance_start + 2) = estimated_force_w_.z();
  state.at(disturbance_start + 3) = lever_arm_torque_b_.x();
  state.at(disturbance_start + 4) = lever_arm_torque_b_.y();
  state.at(disturbance_start + 5) = lever_arm_torque_b_.z();
  return state;
}

void nmpc::TiltMtServoForceImpNMPC::allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i,
                                                 const tf::Quaternion& ref_quat_ib, const tf::Vector3& ref_omega_b,
                                                 const VectorXd& ref_wrench_b, vector<double>& x, vector<double>& u)
{
  VectorXd balanced_wrench_b = ref_wrench_b;
  const tf::Vector3 estimated_force_b = tf::Matrix3x3(ref_quat_ib).inverse() * estimated_force_w_;

  balanced_wrench_b(0) -= estimated_force_b.x();
  balanced_wrench_b(1) -= estimated_force_b.y();
  balanced_wrench_b(2) -= estimated_force_b.z();
  balanced_wrench_b(3) -= lever_arm_torque_b_.x();
  balanced_wrench_b(4) -= lever_arm_torque_b_.y();
  balanced_wrench_b(5) -= lever_arm_torque_b_.z();

  TiltMtServoNMPC::allocateToXU(ref_pos_i, ref_vel_i, ref_quat_ib, ref_omega_b, balanced_wrench_b, x, u);
}

void nmpc::TiltMtServoForceImpNMPC::publishModeledDisturbanceWrench() const
{
  geometry_msgs::WrenchStamped wrench;
  wrench.header.frame_id = nh_.getNamespace().substr(1) + "/cog";
  wrench.header.stamp = ros::Time::now();

  const tf::Matrix3x3 rotation_wb = estimator_->getOrientation(Frame::COG, estimate_mode_);
  const tf::Vector3 force_b = rotation_wb.inverse() * estimated_force_w_;
  tf::vector3TFToMsg(force_b, wrench.wrench.force);
  tf::vector3TFToMsg(lever_arm_torque_b_, wrench.wrench.torque);
  pub_disturb_wrench_.publish(wrench);
}

void nmpc::TiltMtServoForceImpNMPC::sendCmd()
{
  TiltMtServoNMPC::sendCmd();
  publishModeledDisturbanceWrench();
}

void nmpc::TiltMtServoForceImpNMPC::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if (!config.nmpc_flag)
    return;

  const auto solver = boost::dynamic_pointer_cast<ForceImpSolver>(mpc_solver_ptr_);
  if (!solver)
  {
    ROS_ERROR("The MPC solver is not the CoG force-impedance model.");
    return;
  }

  try
  {
    switch (level)
    {
      case Levels::RECONFIGURE_NMPC_ENLARGE_FACTOR:
        solver->setEnlargeFactor(config.enlarge_factor);
        break;
      case Levels::RECONFIGURE_NMPC_Q_P_XY:
        solver->setForceImpedanceWeight("pKx", config.Qp_xy, false);
        solver->setForceImpedanceWeight("pKy", config.Qp_xy);
        break;
      case Levels::RECONFIGURE_NMPC_Q_P_Z:
        solver->setForceImpedanceWeight("pKz", config.Qp_z);
        break;
      case Levels::RECONFIGURE_NMPC_Q_V_XY:
        solver->setForceImpedanceWeight("pDx", config.Qv_xy, false);
        solver->setForceImpedanceWeight("pDy", config.Qv_xy);
        break;
      case Levels::RECONFIGURE_NMPC_Q_V_Z:
        solver->setForceImpedanceWeight("pDz", config.Qv_z);
        break;
      case Levels::RECONFIGURE_NMPC_PM_XY:
        solver->setForceImpedanceWeight("pMx", config.pMxy, false);
        solver->setForceImpedanceWeight("pMy", config.pMxy, false);
        setForceImpedanceParams();
        break;
      case Levels::RECONFIGURE_NMPC_PM_Z:
        solver->setForceImpedanceWeight("pMz", config.pMz, false);
        setForceImpedanceParams();
        break;
      case Levels::RECONFIGURE_NMPC_Q_Q_XY:
        mpc_solver_ptr_->setCostWDiagElement(7, config.Qq_xy);
        mpc_solver_ptr_->setCostWDiagElement(8, config.Qq_xy);
        break;
      case Levels::RECONFIGURE_NMPC_Q_Q_Z:
        mpc_solver_ptr_->setCostWDiagElement(9, config.Qq_z);
        break;
      case Levels::RECONFIGURE_NMPC_Q_W_XY:
        mpc_solver_ptr_->setCostWDiagElement(10, config.Qw_xy);
        mpc_solver_ptr_->setCostWDiagElement(11, config.Qw_xy);
        break;
      case Levels::RECONFIGURE_NMPC_Q_W_Z:
        mpc_solver_ptr_->setCostWDiagElement(12, config.Qw_z);
        break;
      case Levels::RECONFIGURE_NMPC_Q_A:
        for (int i = 13; i < 13 + joint_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Qa);
        break;
      case Levels::RECONFIGURE_NMPC_R_T:
        for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Rt, false);
        break;
      case Levels::RECONFIGURE_NMPC_R_AC_D:
        for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
          mpc_solver_ptr_->setCostWDiagElement(i, config.Rac_d, false);
        break;
      default:
        ROS_INFO("The dynamic-reconfigure variable is not used by force-impedance NMPC.");
        break;
    }
  }
  catch (const std::invalid_argument& exception)
  {
    ROS_ERROR_STREAM("NMPC force-impedance config failed: " << exception.what());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoForceImpNMPC, aerial_robot_control::ControlBase)
