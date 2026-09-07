from pathlib import Path
import re
import unittest


WORKSPACE = Path(__file__).resolve().parents[3]

COMPONENTS = {
    "openflex_drivers": "openflex_drivers",
    "openflex_vr_apk": "openflex_vr_apk",
    "OpenFleX": "OpenFleX",
    "openflex_EXO": "openflex_EXO",
    "openflex_mujoco": "openflex_mujoco",
    "openflex_vr_bridge": "openflex_vr_bridge",
    "openflex_vla": "openflex_vla",
    "openarmx_description": "openflex_armx/openarmx_description",
    "openarmx_ros2": "openflex_armx/openarmx_ros2",
    "openarmx_teleop_vr": "openflex_armx/openarmx_teleop_vr",
    "openarmx_tools": "openflex_armx/openarmx_tools",
    "openarmx_hands": "openflex_armx/openarmx_hands",
    "base_model_interface_layer": "openflex_chassis/base_model_interface_layer",
    "hardware_sensor_layer": "openflex_chassis/hardware_sensor_layer",
    "mapping_localization_layer": "openflex_chassis/mapping_localization_layer",
    "motion_control_layer": "openflex_chassis/motion_control_layer",
    "navigation_layer": "openflex_chassis/navigation_layer",
    "system_bringup_layer": "openflex_chassis/system_bringup_layer",
    "tools_common_layer": "openflex_chassis/tools_common_layer",
    "openarmx_head_bringup": "openflex_head/openarmx_head_bringup",
    "openarmx_head_description": "openflex_head/openarmx_head_description",
    "openarmx_head_hardware": "openflex_head/openarmx_head_hardware",
    "openarmx_head_teleop_vr_pico": "openflex_head/openarmx_head_teleop_vr_pico",
    "openarmx_head_tools": "openflex_head/openarmx_head_tools",
    "openarmx_head_visio_h264": "openflex_head/openarmx_head_visio_h264",
    "openarmx_integrated_bringup": "openflex_integrated/openarmx_integrated_bringup",
    "openarmx_integrated_description": "openflex_integrated/openarmx_integrated_description",
    "openflex_gui": "openflex_integrated/openflex_gui",
    "openflex_manager": "openflex_integrated/openflex_manager",
    "lift_slide_bringup": "openflex_lift_slide/lift_slide_bringup",
    "lift_slide_description": "openflex_lift_slide/lift_slide_description",
    "lift_slide_driver": "openflex_lift_slide/lift_slide_driver",
    "lift_slide_msgs": "openflex_lift_slide/lift_slide_msgs",
    "lift_slide_panel": "openflex_lift_slide/lift_slide_panel",
}


class ComponentManifestTest(unittest.TestCase):
    def test_environment_installs_ros2_control_cli(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("ros-humble-ros2controlcli", text)

    def test_environment_installs_python_yaml_for_dexterous_hands(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("python3-yaml", text)

    def test_installer_registers_dexterous_hand_and_ik_packages(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        expected_packages = {
            "openarmx_hands",
            "hands_bringup",
            "hands_description",
            "hands_hardware",
            "openarmx_hand_bringup",
            "openarmx_hand_description",
            "openarmx_hand_gui",
            "openarmx_hand_hardware",
            "openarmx_hands_bridge",
            "openarmx_hands_hig",
            "openarmx_ik_control_panel",
        }
        for package in expected_packages:
            with self.subTest(package=package):
                self.assertIn(package, text)

    def test_all_managed_repositories_have_metadata(self):
        for name, relative_path in COMPONENTS.items():
            with self.subTest(name=name):
                base = WORKSPACE.parent if name in {"openflex_drivers", "openflex_vr_apk"} else WORKSPACE / "src"
                manifest = base / relative_path / "openflex_component.yaml"
                self.assertTrue(manifest.is_file(), manifest)
                text = manifest.read_text(encoding="utf-8")
                self.assertRegex(text, rf"(?m)^\s*name:\s*{re.escape(name)}\s*$")
                self.assertRegex(text, r"(?m)^\s*branch:\s*v[0-9][^\s]*\s*$")
                self.assertRegex(text, r"(?m)^\s*revision:\s*[0-9]+(?:\.[0-9]+)*\s*$")
                self.assertIn("required_files:", text)
                if name == "openflex_vla":
                    self.assertRegex(text, r"(?m)^\s*update_policy:\s*download_if_missing\s*$")


if __name__ == "__main__":
    unittest.main()
