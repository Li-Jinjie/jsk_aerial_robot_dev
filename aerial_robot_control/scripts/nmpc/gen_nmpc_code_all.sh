#!/bin/bash

# ===== avoid jobserver warning when "catkin build", given by ChatGPT o3
set -e
unset MAKEFLAGS
# =====

STAMP_FILE="../../include/aerial_robot_control/nmpc/dragon_gimbal_servo_dist_mdl/c_generated_code/.source_hash"
SOURCE_HASH=$(
    {
        find nmpc_tilt_mt -type f -name '*.py' -print0 | sort -z | xargs -0 sha256sum
        sha256sum gen_nmpc_code.py gen_nmpc_code_all.sh ../../../robots/dragon/config/DragonNMPCGimbalServoDist.yaml
    } | sha256sum | cut -d' ' -f1
)

REQUIRED_SOLVERS=(
    "../../include/aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/c_generated_code/libacados_ocp_solver_fix_qd_thrust_out_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_no_servo_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_no_servo_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_dist_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_dist_cog_force_imp_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_dist_cog_force_imp_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_dist_imp_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_dist_imp_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_thrust_dist_mdl.so"
    "../../include/aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_imp_mdl/c_generated_code/libacados_ocp_solver_tilt_qd_servo_thrust_dist_imp_mdl.so"
    "../../include/aerial_robot_control/nmpc/dragon_gimbal_servo_dist_mdl/c_generated_code/libacados_ocp_solver_dragon_gimbal_servo_dist_mdl.so"
)

if [[ -f "$STAMP_FILE" ]] && [[ "$(tr -d '\n' < "$STAMP_FILE")" == "$SOURCE_HASH" ]]
then
    all_solvers_exist=true
    for solver in "${REQUIRED_SOLVERS[@]}"
    do
        if [[ ! -f "$solver" ]]
        then
            all_solvers_exist=false
            break
        fi
    done
    if [[ "$all_solvers_exist" == true ]]
    then
        echo "acados NMPC code is up to date."
        exit 0
    fi
fi

MODELS=(
    NMPCFixQdThrustOut
    NMPCTiltQdNoServo
    NMPCTiltQdServo
    NMPCTiltQdServoDist
    NMPCTiltQdServoCoGForceImpedance
    NMPCTiltQdServoImpedance
    NMPCTiltQdServoThrustDist
    NMPCTiltQdServoThrustImpedance
    NMPCDragonGimbalServoDist
#    NMPCTiltTriServo
#    NMPCTiltBiServoDist
#    NMPCTiltBi2OrdServo
#    MHEWrenchEstAccMom
)

for model in "${MODELS[@]}"
do
    echo "Generating NMPC code for model: $model"
    python3 gen_nmpc_code.py -m "$model"
done

echo "$SOURCE_HASH" > "$STAMP_FILE"
