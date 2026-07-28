#pragma once

#include <aerial_robot_control/nmpc/tilt_mt_servo_dist_nmpc_controller.h>
#include <dragon/model/full_vectoring_robot_model.h>

#include <array>

namespace aerial_robot_control
{
class DragonGimbalServoDistNMPC : public nmpc::TiltMtServoDistNMPC
{
public:
  DragonGimbalServoDistNMPC() = default;
  ~DragonGimbalServoDistNMPC() override = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;

protected:
  void initNMPCParams() override;
  void initAllocMat() override;
  void prepareNMPCParams() override;
  void allocateToXU(const tf::Vector3& ref_pos_i, const tf::Vector3& ref_vel_i, const tf::Quaternion& ref_quat_ib,
                    const tf::Vector3& ref_omega_b, const Eigen::VectorXd& ref_wrench_b, std::vector<double>& x,
                    std::vector<double>& u) override;
  void callbackJointStates(const sensor_msgs::JointStateConstPtr& msg) override;
  std::string getServoJointName(int index) const override;

private:
  static constexpr int kModuleCount = 4;
  static constexpr int kGimbalCount = 2 * kModuleCount;
  static constexpr int kPhysicalParameterCount = 38;

  boost::shared_ptr<Dragon::FullVectoringRobotModel> dragon_robot_model_;
  const std::array<std::string, kGimbalCount> gimbal_joint_names_{ "gimbal1_roll",  "gimbal1_pitch", "gimbal2_roll",
                                                                   "gimbal2_pitch", "gimbal3_roll",  "gimbal3_pitch",
                                                                   "gimbal4_roll",  "gimbal4_pitch" };

  std::vector<double> dragonPhysicalParams() const;
};

}  // namespace aerial_robot_control
