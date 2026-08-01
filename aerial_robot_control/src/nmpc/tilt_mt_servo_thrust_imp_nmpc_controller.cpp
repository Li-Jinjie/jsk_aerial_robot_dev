//
// Created by li-jinjie on 24-11-21.
//

#include "aerial_robot_control/nmpc/tilt_mt_servo_thrust_imp_nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltMtServoThrustImpNMPC::initNMPCCostW()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* control parameters with dynamic reconfigure */
  double pMxy, pMz, Qv_xy, Qv_z, Qp_xy, Qp_z, oMxy, oMz, Qw_xy, Qw_z, Qq_xy, Qq_z, Qa, Qt, Rtc_d, Rac_d;
  getNMPCTunableParam<double>(nmpc_nh, "pMxy", pMxy, 1.5);
  getNMPCTunableParam<double>(nmpc_nh, "pMz", pMz, 1.5);
  getNMPCIntTunableParam(nmpc_nh, "Qv_xy", Qv_xy, 10);
  getNMPCIntTunableParam(nmpc_nh, "Qv_z", Qv_z, 10);
  getNMPCIntTunableParam(nmpc_nh, "Qp_xy", Qp_xy, 6);
  getNMPCIntTunableParam(nmpc_nh, "Qp_z", Qp_z, 6);

  getNMPCTunableParam<double>(nmpc_nh, "oMxy", oMxy, 0.9);
  getNMPCTunableParam<double>(nmpc_nh, "oMz", oMz, 0.9);
  getNMPCIntTunableParam(nmpc_nh, "Qw_xy", Qw_xy, 10);
  getNMPCIntTunableParam(nmpc_nh, "Qw_z", Qw_z, 10);
  getNMPCIntTunableParam(nmpc_nh, "Qq_xy", Qq_xy, 6);
  getNMPCIntTunableParam(nmpc_nh, "Qq_z", Qq_z, 6);

  getNMPCIntTunableParam(nmpc_nh, "Qa", Qa, 0);
  getNMPCIntTunableParam(nmpc_nh, "Qt", Qt, 0);

  getNMPCIntTunableParam(nmpc_nh, "Rtc_d", Rtc_d, 1);
  getNMPCIntTunableParam(nmpc_nh, "Rac_d", Rac_d, 250);

  // diagonal matrix
  for (int i = 13; i < 13 + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qa);
  for (int i = 13 + joint_num_; i < 13 + joint_num_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Qt);

  for (int i = mpc_solver_ptr_->NX_; i < mpc_solver_ptr_->NX_ + motor_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rtc_d, false);
  for (int i = mpc_solver_ptr_->NX_ + motor_num_; i < mpc_solver_ptr_->NX_ + motor_num_ + joint_num_; ++i)
    mpc_solver_ptr_->setCostWDiagElement(i, Rac_d, false);

  // impedance matrix
  auto imp_mpc_solver_ptr =
      boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
  if (imp_mpc_solver_ptr)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pMx", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMy", pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMz", pMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDx", Qv_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDy", Qv_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDz", Qv_z, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKx", Qp_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKy", Qp_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKz", Qp_z, false);

    imp_mpc_solver_ptr->setImpedanceWeight("oMx", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMy", oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMz", oMz, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDx", Qw_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDy", Qw_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDz", Qw_z, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKx", Qq_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKy", Qq_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKz", Qq_z, true);  // update W and WN
  }
  else
  {
    ROS_ERROR("The MPC solver is not the impedance model. Please check the MPC solver!!!!");
  }
}

void nmpc::TiltMtServoThrustImpNMPC::setImpParams()
{
  auto imp_mpc_solver_ptr =
      boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
  if (!imp_mpc_solver_ptr)
  {
    ROS_ERROR("The MPC solver is not the impedance model. Please check the MPC solver!!!!");
    return;
  }

  const double* pM = imp_mpc_solver_ptr->getpM();
  const double* oM = imp_mpc_solver_ptr->getoM();
  vector<double> p = { pM[0], pM[1], pM[2], oM[0], oM[1], oM[2] };
  mpc_solver_ptr_->setParameters(p, idx_p_dist_end_ + 1);

  idx_p_imp_end_ = idx_p_dist_end_ + 6;
}

void nmpc::TiltMtServoThrustImpNMPC::initNMPCParams()
{
  TiltMtServoThrustDistNMPC::initNMPCParams();
  setImpParams();
}

nmpc::NMPCConfigMask nmpc::TiltMtServoThrustImpNMPC::getSupportedNMPCConfigMask() const
{
  using namespace NMPCConfigFields;
  return QP_XY | QP_Z | QV_XY | QV_Z | PM_XY | PM_Z | QQ_XY | QQ_Z | QW_XY | QW_Z | OM_XY | OM_Z | QA | QT | RTC_D |
         RAC_D;
}

void nmpc::TiltMtServoThrustImpNMPC::applyNMPCConfig(const NMPCConfig& config, NMPCConfigMask mask)
{
  using namespace NMPCConfigFields;
  auto imp_mpc_solver_ptr =
      boost::dynamic_pointer_cast<mpc_solver::TiltQdServoThrustDistImpMdlMPCSolver>(mpc_solver_ptr_);
  if (!imp_mpc_solver_ptr)
  {
    throw std::runtime_error("The MPC solver is not the thrust-impedance model.");
  }

  if (mask & QP_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pKx", config.Qp_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pKy", config.Qp_xy, true);
  }
  if (mask & QP_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("pKz", config.Qp_z, true);
  if (mask & QV_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pDx", config.Qv_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pDy", config.Qv_xy, true);
  }
  if (mask & QV_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("pDz", config.Qv_z, true);
  if (mask & PM_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("pMx", config.pMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("pMy", config.pMxy, true);
  }
  if (mask & PM_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("pMz", config.pMz, true);
  if (mask & QQ_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("oKx", config.Qq_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oKy", config.Qq_xy, true);
  }
  if (mask & QQ_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("oKz", config.Qq_z, true);
  if (mask & QW_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("oDx", config.Qw_xy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oDy", config.Qw_xy, true);
  }
  if (mask & QW_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("oDz", config.Qw_z, true);
  if (mask & OM_XY)
  {
    imp_mpc_solver_ptr->setImpedanceWeight("oMx", config.oMxy, false);
    imp_mpc_solver_ptr->setImpedanceWeight("oMy", config.oMxy, true);
  }
  if (mask & OM_Z)
    imp_mpc_solver_ptr->setImpedanceWeight("oMz", config.oMz, true);

  TiltMtServoThrustDistNMPC::applyNMPCConfig(config, mask & (QA | QT | RTC_D | RAC_D));
  setImpParams();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltMtServoThrustImpNMPC, aerial_robot_control::ControlBase)
