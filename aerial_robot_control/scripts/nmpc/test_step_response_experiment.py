import unittest

import numpy as np

from argparse import Namespace

import transformations as tf

from nmpc_tilt_mt.utils.step_response_experiment import base30_cases, base36_cases, base90_cases, scalar_step_metrics
from sim_nmpc import get_step_response_target


class StepResponseExperimentTest(unittest.TestCase):
    def test_zero_workpoint_matrix_has_30_expected_cases(self):
        cases = base30_cases()
        self.assertEqual(len(cases), 30)
        self.assertEqual(len({case.slug for case in cases}), 30)
        self.assertEqual({case.workpoint_rpy_deg for case in cases}, {(0.0, 0.0, 0.0)})
        self.assertEqual({case.axis for case in cases}, {"x", "y", "z", "roll", "pitch", "yaw"})
        self.assertEqual(
            {case.amplitude for case in cases if case.axis in ("x", "y", "z")},
            {0.2, 0.4, 0.6, 0.8, 1.0},
        )
        self.assertEqual(
            {case.amplitude for case in cases if case.axis in ("roll", "pitch", "yaw")},
            {10.0, 30.0, 50.0, 70.0, 90.0},
        )

    def test_base_matrix_has_36_unique_cases(self):
        cases = base36_cases()
        self.assertEqual(len(cases), 36)
        self.assertEqual(len({case.slug for case in cases}), 36)

    def test_expanded_matrix_has_90_unique_cases(self):
        cases = base90_cases()
        self.assertEqual(len(cases), 90)
        self.assertEqual(len({case.slug for case in cases}), 90)

    def test_attitude_step_is_composed_in_workpoint_frame(self):
        args = Namespace(
            workpoint_rpy_deg=[30.0, 30.0, 0.0],
            step_time=2.0,
            step_axis="roll",
            step_amplitude=90.0,
        )
        _, target_rpy, active = get_step_response_target(args, 2.0)
        actual = tf.euler_matrix(*target_rpy.flatten(), axes="sxyz")
        workpoint = tf.euler_matrix(*np.radians(args.workpoint_rpy_deg), axes="sxyz")
        expected = workpoint @ tf.rotation_matrix(np.radians(90.0), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        self.assertEqual(active, 1)

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
