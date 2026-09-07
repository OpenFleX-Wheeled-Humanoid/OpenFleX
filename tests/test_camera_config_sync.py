from pathlib import Path
import os
import subprocess
import tempfile
import unittest


INSTALLER = Path(__file__).resolve().parents[1] / "install_openflex_drivers_and_build.sh"


def camera_config(name: str, fps: int) -> str:
    return f'''camera_config_name: {name}
version: 1

defaults:
  fps: {fps}

cameras:
  right_wrist:
    camera_model: D405
    serial_no: "old-right"
    launch_fps: {fps}
  left_wrist:
    camera_model: D405
    serial_no: "old-left"
    launch_fps: {fps}
  head:
    camera_model: D435i
    serial_no: "old-head"
    enable_imu: false
  base:
    camera_model: D435i
    serial_no: "old-base"
    enable_imu: false
'''


def expected_updated_config(text: str) -> str:
    return (
        text.replace('serial_no: "old-right"', 'serial_no: "new-right"')
        .replace('serial_no: "old-head"', 'serial_no: "new-head"')
        .replace('serial_no: "old-base"', 'serial_no: "new-base"')
    )


class CameraConfigSyncTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name)
        self.home = self.root / "home"
        self.workspace = self.root / "openflex_ws"
        self.camera_dir = self.workspace / "src/openflex_vla/config/cameras"
        self.user_config = self.home / ".openflex/cameras_config.yaml"
        self.default_config = self.camera_dir / "cameras_config.yaml"
        self.fps30_config = self.camera_dir / "cameras_config_30fps.yaml"
        self.camera_dir.mkdir(parents=True)
        self.user_config.parent.mkdir(parents=True)
        self.user_config.write_text(camera_config("user", 15), encoding="utf-8")
        self.default_config.write_text(camera_config("default", 15), encoding="utf-8")
        self.fps30_config.write_text(camera_config("fps30", 30), encoding="utf-8")

    def tearDown(self):
        self.temp_dir.cleanup()

    def run_camera_config(self) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment["HOME"] = str(self.home)
        environment["OPENFLEX_WORKSPACE"] = str(self.workspace)
        return subprocess.run(
            ["bash", str(INSTALLER), "--camera"],
            input="new-right\n\nnew-head\nnew-base\n",
            text=True,
            capture_output=True,
            env=environment,
            check=False,
        )

    def test_updates_only_selected_serials_in_all_camera_configs(self):
        originals = {
            path: path.read_text(encoding="utf-8")
            for path in (self.user_config, self.default_config, self.fps30_config)
        }

        result = self.run_camera_config()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for path, original in originals.items():
            with self.subTest(path=path):
                self.assertEqual(
                    path.read_text(encoding="utf-8"),
                    expected_updated_config(original),
                )

    def test_missing_package_config_is_skipped_without_blocking_other_files(self):
        self.fps30_config.unlink()
        originals = {
            path: path.read_text(encoding="utf-8")
            for path in (self.user_config, self.default_config)
        }

        result = self.run_camera_config()

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for path, original in originals.items():
            with self.subTest(path=path):
                self.assertEqual(
                    path.read_text(encoding="utf-8"),
                    expected_updated_config(original),
                )
        self.assertIn("跳过不存在的相机配置文件", result.stdout)


if __name__ == "__main__":
    unittest.main()
