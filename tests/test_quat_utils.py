import importlib.util
import unittest


@unittest.skipUnless(
    importlib.util.find_spec("numpy") is not None,
    "numeric dependencies are not installed",
)
class QuaternionForwardKinematicsTests(unittest.TestCase):
    def test_identity_rotations_accumulate_local_positions(self):
        import numpy as np

        from movin_sdk_python.utils.quat_utils import quat_fk

        local_rotations = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0, 0.0],
            ]
        )
        local_positions = np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 2.0, 0.0],
                [0.0, 0.0, 3.0],
            ]
        )

        global_rotations, global_positions = quat_fk(
            local_rotations,
            local_positions,
            [-1, 0, 1],
        )

        np.testing.assert_allclose(global_rotations, local_rotations)
        np.testing.assert_allclose(
            global_positions,
            np.array(
                [
                    [1.0, 0.0, 0.0],
                    [1.0, 2.0, 0.0],
                    [1.0, 2.0, 3.0],
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
