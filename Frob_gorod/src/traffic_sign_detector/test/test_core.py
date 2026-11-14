import unittest

from traffic_sign_detector.core import (
    DistanceEstimator,
    SignSpec,
    required_confidence,
)


SPECS = {
    "straight": SignSpec(0.100, "height"),
    "parking": SignSpec(0.090, "mean"),
    "obstacle": SignSpec(0.130, "width"),
}


def make_estimator(filter_size=5):
    return DistanceEstimator(
        focal_length_px=900.0,
        parking_distance_scale=1.54,
        filter_size=filter_size,
        sign_specs=SPECS,
    )


class DistanceEstimatorTest(unittest.TestCase):
    def test_height_distance_is_returned_in_centimetres(self):
        estimator = make_estimator()
        self.assertAlmostEqual(
            estimator.estimate_filtered_cm("straight", 50, 90),
            100.0,
        )

    def test_parking_scale_is_applied_to_mean_axis(self):
        estimator = make_estimator()
        expected_cm = 0.090 * 900.0 / 90.0 * 1.54 * 100.0
        self.assertAlmostEqual(
            estimator.estimate_filtered_cm("parking", 80, 100),
            expected_cm,
        )

    def test_width_axis_is_used_for_obstacle(self):
        estimator = make_estimator()
        expected_cm = 0.130 * 900.0 / 117.0 * 100.0
        self.assertAlmostEqual(
            estimator.estimate_filtered_cm("obstacle", 117, 30),
            expected_cm,
        )

    def test_filter_returns_median_for_each_class(self):
        estimator = make_estimator(filter_size=3)
        estimator.estimate_filtered_cm("straight", 10, 90)
        estimator.estimate_filtered_cm("straight", 10, 45)
        filtered = estimator.estimate_filtered_cm("straight", 10, 30)
        self.assertAlmostEqual(filtered, 200.0)

    def test_unknown_class_has_no_distance(self):
        estimator = make_estimator()
        self.assertIsNone(estimator.estimate_filtered_cm("unknown", 100, 100))

    def test_class_specific_confidence(self):
        self.assertEqual(
            required_confidence("bus_stop", 0.8, 0.4, 0.6),
            0.4,
        )
        self.assertEqual(
            required_confidence("parking", 0.8, 0.4, 0.6),
            0.6,
        )
        self.assertEqual(
            required_confidence("left", 0.8, 0.4, 0.6),
            0.8,
        )


if __name__ == "__main__":
    unittest.main()
