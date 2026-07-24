#ifndef TILT_MT_SERVO_FORCE_IMP_NMPC_CONTROLLER_H
#define TILT_MT_SERVO_FORCE_IMP_NMPC_CONTROLLER_H

#include "aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_cog_force_imp_mdl/nmpc_solver.h"

namespace aerial_robot_control
{
namespace nmpc
{

class TiltMtServoForceImpNMPC : public TiltMtServoDistNMPC
{
protected:
  tf::Vector3 estimated_force_w_{ 0.0, 0.0, 0.0 };
  tf::Vector3 lever_arm_torque_b_{ 0.0, 0.0, 0.0 };

  void initNMPCCostW() override;
  void initNMPCParams() override;

  void controlCore(bool is_warmup = false) override;
  void sendCmd() override;
  std::vector<double> meas2VecX(bool is_modified_by_traj_frame) override;
  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const VectorXd& ref_wrench_b, vector<double>& x,
                    vector<double>& u) override;

  void cfgNMPCCallback(NMPCConfig& config, uint32_t level) override;

  void setForceImpedanceParams();
  void updateForceAndLeverArmTorque();
  void publishModeledDisturbanceWrench() const;
};

}  // namespace nmpc
}  // namespace aerial_robot_control

#endif  // TILT_MT_SERVO_FORCE_IMP_NMPC_CONTROLLER_H
