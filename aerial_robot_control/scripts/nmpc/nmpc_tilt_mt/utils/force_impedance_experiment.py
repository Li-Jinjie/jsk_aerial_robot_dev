import json
import os

import numpy as np


SCENARIO_NAME = "force-impedance-compare"
SCENARIO_DURATION = 18.0
STEADY_STATE_WINDOWS = ((6.0, 7.0), (11.0, 12.0), (16.0, 17.0))


def get_force_comparison_wrench(t):
    """Return [force_world, torque_ee] for the shared force-only scenario."""
    wrench = np.zeros(6)
    if 2.0 <= t < 7.0:
        wrench[0] = 5.0
    elif 7.0 <= t < 12.0:
        wrench[0:2] = [5.0, -5.0]
    elif 12.0 <= t < 17.0:
        wrench[0:3] = [5.0, -5.0, -5.0]
    return wrench


def impedance_parameters(params):
    return {
        "mass": [params["pMxy"], params["pMxy"], params["pMz"]],
        "damping": [params["Qv_xy"], params["Qv_xy"], params["Qv_z"]],
        "stiffness": [params["Qp_xy"], params["Qp_xy"], params["Qp_z"]],
    }


def save_run_bundle(path, metadata, **arrays):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    np.savez_compressed(path, metadata=json.dumps(metadata, sort_keys=True), **arrays)
    print(f"Run bundle saved to {path}")


def load_run_bundle(path):
    data = np.load(path, allow_pickle=False)
    metadata = json.loads(str(data["metadata"].item()))
    return data, metadata
