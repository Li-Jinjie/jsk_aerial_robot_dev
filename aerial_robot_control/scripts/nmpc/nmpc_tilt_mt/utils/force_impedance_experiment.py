import json
import os
from datetime import datetime

import numpy as np


SCENARIO_NAME = "force-impedance-compare"
SCENARIO_DURATION = 20.0
SCENARIO_EVENT_TIMES = (2.0, 7.0, 12.0, 17.0)
STEADY_STATE_WINDOWS = ((6.0, 7.0), (11.0, 12.0), (16.0, 17.0), (19.0, 20.0))

NMPC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
PAPER_RESULTS_ROOT = os.path.join(NMPC_ROOT, "experiment_results", "impedance", "paper")


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


def _number_tag(value):
    return f"{float(value):g}".replace("-", "m").replace(".", "p")


def impedance_run_tag(params):
    """Return a compact filename tag for the active translational impedance setup."""
    return "_".join(
        (
            f"m{_number_tag(params['pMxy'])}-{_number_tag(params['pMz'])}",
            f"d{_number_tag(params['Qv_xy'])}-{_number_tag(params['Qv_z'])}",
            f"k{_number_tag(params['Qp_xy'])}-{_number_tag(params['Qp_z'])}",
            f"ef{_number_tag(params.get('enlarge_factor', 1.0))}",
        )
    )


def default_run_bundle_path(kind, params, descriptor=""):
    """Create a unique paper-data path below experiment_results/impedance/paper."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    parts = [kind]
    if descriptor:
        parts.append(descriptor)
    parts.extend((impedance_run_tag(params), timestamp))
    filename = "_".join(parts) + ".npz"
    return os.path.join(PAPER_RESULTS_ROOT, "data", filename)


def save_run_bundle(path, metadata, **arrays):
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    np.savez_compressed(path, metadata=json.dumps(metadata, sort_keys=True), **arrays)
    print(f"Run bundle saved to {path}")


def load_run_bundle(path):
    data = np.load(path, allow_pickle=False)
    metadata = json.loads(str(data["metadata"].item()))
    return data, metadata
