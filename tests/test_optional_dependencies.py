import os
import subprocess
import sys
import textwrap
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class OptionalDependencyTests(unittest.TestCase):
    def run_without_site_packages(self, source):
        environment = os.environ.copy()
        environment.pop("PYTHONPATH", None)
        return subprocess.run(
            [sys.executable, "-S", "-c", textwrap.dedent(source)],
            cwd=PROJECT_ROOT,
            text=True,
            capture_output=True,
            check=False,
            env=environment,
        )

    def test_core_import_has_no_third_party_dependency(self):
        result = self.run_without_site_packages(
            """
            import sys
            import movin_sdk_python as sdk

            assert sdk.MocapReceiver
            assert sdk.MovinSession
            assert sdk.OscRecorder
            assert "numpy" not in sys.modules
            assert "scipy" not in sys.modules
            assert "mujoco" not in sys.modules
            assert "mink" not in sys.modules
            """
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_dependency_free_utility_modules_remain_available(self):
        result = self.run_without_site_packages(
            """
            import movin_sdk_python.utils
            from movin_sdk_python.utils.skeleton_presets import get_preset

            assert get_preset("movinman").name == "movinman"
            """
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_retargeting_error_names_install_extra(self):
        result = self.run_without_site_packages(
            """
            import movin_sdk_python as sdk

            try:
                sdk.Retargeter
            except sdk.MissingOptionalDependencyError as exc:
                assert exc.extra == "retargeting"
                assert "movin_sdk_python[retargeting]" in str(exc)
            else:
                raise AssertionError("Retargeter unexpectedly imported without site packages")
            """
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_viewer_error_names_install_extra(self):
        result = self.run_without_site_packages(
            """
            import movin_sdk_python as sdk

            for name in ("MocapViewer", "MujocoViewer"):
                try:
                    getattr(sdk, name)
                except sdk.MissingOptionalDependencyError as exc:
                    assert exc.extra == "viewer"
                    assert "movin_sdk_python[viewer]" in str(exc)
                else:
                    raise AssertionError(
                        f"{name} unexpectedly imported without site packages"
                    )
            """
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_numeric_utility_error_names_install_extra(self):
        result = self.run_without_site_packages(
            """
            import movin_sdk_python.utils as utils
            from movin_sdk_python import MissingOptionalDependencyError

            try:
                utils.quat_mul
            except MissingOptionalDependencyError as exc:
                assert exc.extra == "retargeting"
                assert "movin_sdk_python[retargeting]" in str(exc)
            else:
                raise AssertionError("Numeric utility imported without site packages")
            """
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
