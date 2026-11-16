#ifndef TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NMPC_SOLVER_H
#define TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NMPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_cog_force_imp_mdl/c_generated_code/acados_solver_tilt_qd_servo_dist_cog_force_imp_mdl.h"

#include <Eigen/Dense>

namespace aerial_robot_control
{
namespace mpc_solver
{

class TiltQdServoDistCoGForceImpMdlMPCSolver : public BaseMPCSolver
{
public:
  TiltQdServoDistCoGForceImpMdlMPCSolver()
  {
    NN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_N;
    NX_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NX;
    NZ_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NZ;
    NU_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NU;
    NP_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NP;
    NBX_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NBX;
    NBX0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NBX0;
    NBU_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NBU;
    NSBX_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSBX;
    NSBU_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSBU;
    NSH_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSH;
    NSH0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSH0;
    NSG_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSG;
    NSPHI_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSPHI;
    NSHN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSHN;
    NSGN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSGN;
    NSPHIN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSPHIN;
    NSPHI0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSPHI0;
    NSBXN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSBXN;
    NS_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NS;
    NS0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NS0;
    NSN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NSN;
    NG_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NG;
    NBXN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NBXN;
    NGN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NGN;
    NY0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NY0;
    NY_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NY;
    NYN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NYN;
    NH_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NH;
    NHN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NHN;
    NH0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NH0;
    NPHI0_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NPHI0;
    NPHI_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NPHI;
    NPHIN_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NPHIN;
    NR_ = TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NR;

    acados_ocp_capsule_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_create_capsule();

    const int status = tilt_qd_servo_dist_cog_force_imp_mdl_acados_create(acados_ocp_capsule_);
    if (status)
      throw std::runtime_error("tilt_qd_servo_dist_cog_force_imp_mdl_acados_create() returned status " +
                               std::to_string(status) + ". Exiting.");

    nlp_config_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = tilt_qd_servo_dist_cog_force_imp_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  }

  ~TiltQdServoDistCoGForceImpMdlMPCSolver() override
  {
    int status = tilt_qd_servo_dist_cog_force_imp_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_dist_cog_force_imp_mdl_acados_free() returned status " << status << ".\n";

    status = tilt_qd_servo_dist_cog_force_imp_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_dist_cog_force_imp_mdl_acados_free_capsule() returned status " << status << ".\n";
  }

  void setEnlargeFactor(const double factor)
  {
    enlarge_factor_ = factor;
    setCostWeightByForceImpedance();
  }

  void setForceImpedanceWeight(const std::string& type, const double value, const bool update_cost = true)
  {
    if (type == "pMx")
      pM_[0] = value;
    else if (type == "pMy")
      pM_[1] = value;
    else if (type == "pMz")
      pM_[2] = value;
    else if (type == "pDx")
      pD_[0] = value;
    else if (type == "pDy")
      pD_[1] = value;
    else if (type == "pDz")
      pD_[2] = value;
    else if (type == "pKx")
      pK_[0] = value;
    else if (type == "pKy")
      pK_[1] = value;
    else if (type == "pKz")
      pK_[2] = value;
    else
      throw std::invalid_argument("Invalid force-impedance weight type: " + type);

    if (update_cost)
      setCostWeightByForceImpedance();
  }

  const double* getVirtualMass() const
  {
    return pM_;
  }

protected:
  tilt_qd_servo_dist_cog_force_imp_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  double pM_[3] = { 1.0, 1.0, 1.0 };
  double pD_[3] = { 0.0, 0.0, 0.0 };
  double pK_[3] = { 0.0, 0.0, 0.0 };
  double enlarge_factor_ = 1.0;

  void setCostWeightByForceImpedance(const bool set_terminal_cost = true)
  {
    if (W_.size() != static_cast<size_t>(NY_ * NY_))
      throw std::length_error("W size is not equal to NY_ * NY_.");
    if (WN_.size() != static_cast<size_t>(NYN_ * NYN_))
      throw std::length_error("WN size is not equal to NYN_ * NYN_.");

    const Eigen::Matrix3d pK_imp = Eigen::DiagonalMatrix<double, 3>(pK_[0], pK_[1], pK_[2]);
    const Eigen::Matrix3d pD_imp = Eigen::DiagonalMatrix<double, 3>(pD_[0], pD_[1], pD_[2]);
    const Eigen::Matrix3d pM_imp = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 9, 3> p_weight;
    p_weight.block<3, 3>(0, 0) = pK_imp;
    p_weight.block<3, 3>(3, 0) = pD_imp;
    p_weight.block<3, 3>(6, 0) = pM_imp;
    p_weight *= enlarge_factor_;
    const Eigen::Matrix<double, 9, 9> p_weight_mtx = p_weight * p_weight.transpose();

    Eigen::Map<Eigen::MatrixXd> W_mtx(W_.data(), NY_, NY_);
    W_mtx.block<6, 6>(0, 0) = p_weight_mtx.block<6, 6>(0, 0);
    W_mtx.block<3, 3>(17, 17) = p_weight_mtx.block<3, 3>(6, 6);
    W_mtx.block<6, 3>(0, 17) = p_weight_mtx.block<6, 3>(0, 6);
    W_mtx.block<3, 6>(17, 0) = p_weight_mtx.block<3, 6>(6, 0);
    setCostWeightMid(W_);

    if (set_terminal_cost)
    {
      Eigen::Map<Eigen::MatrixXd> WN_mtx(WN_.data(), NYN_, NYN_);
      WN_mtx = W_mtx.block(0, 0, NYN_, NYN_);
      setCostWeightEnd(WN_);
    }
  }

  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return tilt_qd_servo_dist_cog_force_imp_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return tilt_qd_servo_dist_cog_force_imp_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(),
                                                                            p.data(), n_update);
  }

  inline int acadosSolve() override
  {
    return tilt_qd_servo_dist_cog_force_imp_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    tilt_qd_servo_dist_cog_force_imp_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver
}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_DIST_COG_FORCE_IMP_MDL_NMPC_SOLVER_H
