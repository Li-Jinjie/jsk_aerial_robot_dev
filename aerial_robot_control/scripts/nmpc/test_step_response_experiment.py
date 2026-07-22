import unittest

import numpy as np

from nmpc_tilt_mt.utils.step_response_experiment import base36_cases, scalar_step_metrics


class StepResponseExperimentTest(unittest.TestCase):
    def test_base_matrix_has_36_unique_cases(self):
        cases = base36_cases()
        self.assertEqual(len(cases), 36)
        self.assertEqual(len({case.slug for case in cases}), 36)

    def test_monotonic_positive_step(self):
        time = np.linspace(0.0, 10.0, 10001)
        reference = (time >= 2.0).astype(float)
        response = np.where(time >= 2.0, 1.0 - np.exp(-(time - 2.0)), 0.0)
        metrics = scalar_step_metrics(time, response, reference, 2.0, (9.0, 10.0))
        self.assertAlmostEqual(metrics["rise_time_s"], np.log(9.0), places=3)
        self.assertAlmostEqual(metrics["percentage_overshoot_pct"], 0.0)
        self.assertTrue(np.isfinite(metrics["settling_time_s"]))

    def test_negative_step_uses_signed_progress(self):
        time = np.linspace(0.0, 10.0, 10001)
        reference = -(time >= 2.0).astype(float)
        response = np.where(time >= 2.0, -(1.0 - np.exp(-(time - 2.0))), 0.0)
        metrics = scalar_step_metrics(time, response, reference, 2.0, (9.0, 10.0))
        self.assertAlmostEqual(metrics["rise_time_s"], np.log(9.0), places=3)
        self.assertAlmostEqual(metrics["percentage_overshoot_pct"], 0.0)

    def test_unreached_response_returns_nan_times(self):
        time = np.linspace(0.0, 10.0, 1001)
        reference = (time >= 2.0).astype(float)
        response = 0.5 * reference
        metrics = scalar_step_metrics(time, response, reference, 2.0, (9.0, 10.0))
        self.assertTrue(np.isnan(metrics["rise_time_s"]))
        self.assertTrue(np.isnan(metrics["settling_time_s"]))
        self.assertAlmostEqual(metrics["steady_state_error"], 0.5)


if __name__ == "__main__":
    unittest.main()
