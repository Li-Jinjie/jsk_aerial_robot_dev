//
// NMPC solver wrapper for the DRAGON four-module gimbal model.
//

#ifndef DRAGON_GIMBAL_SERVO_DIST_NMPC_SOLVER_H
#define DRAGON_GIMBAL_SERVO_DIST_NMPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/dragon_gimbal_servo_dist_mdl/c_generated_code/acados_solver_dragon_gimbal_servo_dist_mdl.h"

namespace aerial_robot_control
{
namespace mpc_solver
{
class DragonGimbalServoDistMdlMPCSolver : public BaseMPCSolver
{
public:
  DragonGimbalServoDistMdlMPCSolver()
  {
    NN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_N;
    NX_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NX;
    NZ_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NZ;
    NU_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NU;
    NP_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NP;
    NBX_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NBX;
    NBX0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NBX0;
    NBU_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NBU;
    NSBX_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSBX;
    NSBU_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSBU;
    NSH_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSH;
    NSH0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSH0;
    NSG_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSG;
    NSPHI_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSPHI;
    NSHN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSHN;
    NSGN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSGN;
    NSPHIN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSPHIN;
    NSPHI0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSPHI0;
    NSBXN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSBXN;
    NS_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NS;
    NS0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NS0;
    NSN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NSN;
    NG_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NG;
    NBXN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NBXN;
    NGN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NGN;
    NY0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NY0;
    NY_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NY;
    NYN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NYN;
    NH_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NH;
    NHN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NHN;
    NH0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NH0;
    NPHI0_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NPHI0;
    NPHI_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NPHI;
    NPHIN_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NPHIN;
    NR_ = DRAGON_GIMBAL_SERVO_DIST_MDL_NR;

    acados_ocp_capsule_ = dragon_gimbal_servo_dist_mdl_acados_create_capsule();
    const int status = dragon_gimbal_servo_dist_mdl_acados_create(acados_ocp_capsule_);
    if (status)
      throw std::runtime_error("dragon_gimbal_servo_dist_mdl_acados_create() returned status " +
                               std::to_string(status) + ". Exiting.");

    nlp_config_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = dragon_gimbal_servo_dist_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  }

  ~DragonGimbalServoDistMdlMPCSolver() override
  {
    int status = dragon_gimbal_servo_dist_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "dragon_gimbal_servo_dist_mdl_acados_free() returned status " << status << ".\n";

    status = dragon_gimbal_servo_dist_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "dragon_gimbal_servo_dist_mdl_acados_free_capsule() returned status " << status << ".\n";
  }

protected:
  dragon_gimbal_servo_dist_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return dragon_gimbal_servo_dist_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return dragon_gimbal_servo_dist_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(), p.data(),
                                                                    n_update);
  }

  int acadosSolve() override
  {
    return dragon_gimbal_servo_dist_mdl_acados_solve(acados_ocp_capsule_);
  }

  void acadosPrintStats() override
  {
    dragon_gimbal_servo_dist_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver
}  // namespace aerial_robot_control

#endif  // DRAGON_GIMBAL_SERVO_DIST_NMPC_SOLVER_H
