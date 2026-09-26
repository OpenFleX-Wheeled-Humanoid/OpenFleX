from pathlib import Path
import re
import unittest


WORKSPACE = Path(__file__).resolve().parents[3]

COMPONENTS = {
    "openflex_vr_apk": "openflex_vr_apk",
    "OpenFleX": "OpenFleX",
    "openflex_EXO": "openflex_EXO",
    "openflex_mujoco": "openflex_mujoco",
    "openflex_vr_bridge": "openflex_vr_bridge",
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

DRIVER_PLATFORMS = (
    "22.04-amd64-humble",
    "24.04-arm64-Jazzy-Jetson",
)


class ComponentManifestTest(unittest.TestCase):
    def test_environment_installs_ros2_control_cli(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("ros-humble-ros2controlcli", text)

    def test_environment_has_separate_jazzy_dependency_branch(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("ros-jazzy-ros2controlcli", text)
        self.assertIn("ros-jazzy-realsense2-camera", text)
        self.assertIn("install_jazzy_system_build_dependencies", text)

    def test_driver_payloads_are_separated_by_platform(self):
        driver_root = WORKSPACE.parent / "openflex_drivers"
        for platform in DRIVER_PLATFORMS:
            with self.subTest(platform=platform):
                driver_dir = driver_root / platform
                self.assertEqual(len(list(driver_dir.glob("*.deb"))), 5)
                self.assertEqual(len(list(driver_dir.glob("*.whl"))), 1)

    def test_platform_selection_follows_mode_selection(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("--platform TARGET", text)
        main = text[text.index("main() {"):]
        self.assertLess(main.index("choose_mode"), main.index("select_platform"))
        self.assertIn("environment|openflex)", main)
        self.assertIn('compile)', main)
        self.assertIn("auto_detect_platform", main)
        self.assertNotIn('"${MODE}" == "compile" ]]; then\n    select_platform', main)
        for option in range(1, 10):
            self.assertIn(f"  {option}. ", text[text.index("choose_mode() {"):])

    def test_compile_platform_is_detected_from_host(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        detector = text[text.index("auto_detect_platform() {"):text.index("validate_selected_platform() {")]
        self.assertIn("ubuntu:22.04:amd64", detector)
        self.assertIn('PLATFORM="humble"', detector)
        self.assertIn("ubuntu:24.04:arm64", detector)
        self.assertIn('PLATFORM="jazzy"', detector)
        self.assertIn('! -r "${ROS_SETUP}"', detector)

    def test_platform_prompt_is_chinese_and_uses_bracketed_choices(self):
        installer = WORKSPACE / "src" / "OpenFleX" / "install_openflex_drivers_and_build.sh"
        text = installer.read_text(encoding="utf-8")
        self.assertIn("请选择目标主机平台：", text)
        self.assertIn("[1] Ubuntu 22.04 amd64（ROS 2 Humble）", text)
        self.assertIn("[2] Ubuntu 24.04 arm64 Jetson（ROS 2 Jazzy）", text)
        self.assertIn('请输入平台编号 [1/2]：', text)

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
                base = WORKSPACE.parent if name == "openflex_vr_apk" else WORKSPACE / "src"
                manifest = base / relative_path / "openflex_component.yaml"
                self.assertTrue(manifest.is_file(), manifest)
                text = manifest.read_text(encoding="utf-8")
                self.assertRegex(text, rf"(?m)^\s*name:\s*{re.escape(name)}\s*$")
                self.assertRegex(text, r"(?m)^\s*branch:\s*v[0-9][^\s]*\s*$")
                self.assertRegex(text, r"(?m)^\s*revision:\s*[0-9]+(?:\.[0-9]+)*\s*$")
                self.assertIn("required_files:", text)
if __name__ == "__main__":
    unittest.main()
