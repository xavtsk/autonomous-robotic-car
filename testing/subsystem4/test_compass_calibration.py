"""Exercise the laptop fit with known synthetic fields and missing coverage."""
import math
import unittest

from tools.compass_calibration import fit


def synthetic_sweep(phase):
    samples = []
    for index in range(600):
        z = 1.0 - 2 * (index + .5) / 600
        length = math.sqrt(1 - z*z)
        angle = index * 2.399963229728653 + phase
        sphere = [length * math.cos(angle), length * math.sin(angle), z]
        samples.append(tuple(bias + radius * unit for bias, radius, unit
                             in zip((.03, -.02, .05), (.4, .5, .6), sphere)))
    return samples


class CompassCalibrationTests(unittest.TestCase):
    def test_measured_ellipsoid(self):
        result = fit(synthetic_sweep(0), synthetic_sweep(.57))
        for actual, expected in zip(result['offset_gauss'], (.03, -.02, .05)):
            self.assertAlmostEqual(actual, expected, delta=.01)
        self.assertLess(result['held_out']['p95_relative_radius_error'], .04)

    def test_flat_rotations(self):
        flat = [(x, y, 0.) for x, y, _ in synthetic_sweep(0)]
        with self.assertRaisesRegex(ValueError, "swing"):
            fit(flat, synthetic_sweep(.1))

    def test_independent_check_and_unstable_field(self):
        sweep = synthetic_sweep(.42)
        with self.assertRaisesRegex(ValueError, "independently"):
            fit(sweep, sweep)
        disturbed = [(x + .15, y, z) for x, y, z in sweep]
        with self.assertRaises(ValueError):
            fit(synthetic_sweep(0), disturbed)
