#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOME_OPENFLEX_ROOT="${HOME}/openflex_all"
HOME_WORKSPACE_DIR="${HOME_OPENFLEX_ROOT}/openflex_ws"
if [[ -n "${OPENFLEX_WORKSPACE:-}" ]]; then
  WORKSPACE_DIR="$(cd "${OPENFLEX_WORKSPACE}" && pwd)"
elif [[ -d "${HOME_WORKSPACE_DIR}/src" ]]; then
  WORKSPACE_DIR="${HOME_WORKSPACE_DIR}"
elif [[ -d "${SCRIPT_DIR}/src" ]]; then
  WORKSPACE_DIR="${SCRIPT_DIR}"
elif [[ "$(basename "$(dirname "${SCRIPT_DIR}")")" == "src" ]]; then
  WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
elif [[ -d "${SCRIPT_DIR}/../src" ]]; then
  WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
else
  WORKSPACE_DIR="${SCRIPT_DIR}"
fi
OPENFLEX_ROOT="$(cd "${WORKSPACE_DIR}/.." && pwd)"
DRIVER_DIR="${OPENFLEX_ROOT}/openflex_drivers"
OPENFLEX_GIT_ORG="${OPENFLEX_GIT_ORG:-https://github.com/OpenFleX-Wheeled-Humanoid}"
OPENFLEX_GIT_BRANCH="${OPENFLEX_GIT_BRANCH:-v1.0_basic}"
if [[ -n "${OPENFLEX_INSTALL_ROOT:-}" ]]; then
  BOOTSTRAP_ROOT="$(mkdir -p "${OPENFLEX_INSTALL_ROOT}" && cd "${OPENFLEX_INSTALL_ROOT}" && pwd)"
elif [[ -n "${OPENFLEX_WORKSPACE:-}" ]]; then
  BOOTSTRAP_ROOT="$(cd "${WORKSPACE_DIR}/.." && pwd)"
else
  BOOTSTRAP_ROOT="${HOME_OPENFLEX_ROOT}"
fi
ROS_SETUP="/opt/ros/humble/setup.bash"
BUILD_TYPE="Release"
PARALLEL_WORKERS="2"
PYPI_INDEX_URL="${OPENFLEX_PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
KCAN_SDK_VERSION="${KCAN_SDK_VERSION:-1.2.2}"
KCAN_DKMS_VERSION="${KCAN_DKMS_VERSION:-8.20.0}"
KCAN_SDK_URL="${KCAN_SDK_URL:-https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK/releases/download/v${KCAN_SDK_VERSION}/KH-UCANFD_Linux_SDK.zip}"
KCAN_SDK_ZIP="${KCAN_SDK_ZIP:-${HOME}/KH-UCANFD_Linux_SDK.zip}"
KCAN_SDK_DIR="${KCAN_SDK_DIR:-${HOME}/KH-UCANFD_LinuxSDK-v${KCAN_SDK_VERSION}}"
KCAN_DKMS_NAME="${KCAN_DKMS_NAME:-kcan}"
KCAN_DKMS_SRC="${KCAN_DKMS_SRC:-/usr/src/${KCAN_DKMS_NAME}-${KCAN_DKMS_VERSION}}"
DRY_RUN=0
SKIP_INSTALL=0
SKIP_BUILD=0
SKIP_DESKTOP=0
MODE=""

usage() {
  cat <<'USAGE'
Usage:
  ./install_openflex_drivers_and_build.sh [options]

Options:
  --openflex         Run initial OpenFlex installation and build.
  --desktop          Create the OpenFlex desktop GUI launcher only.
  --vr               Install OpenFlex VR Android software only.
  --camera           Configure OpenFlex camera serial numbers only.
  --lidar            Configure OpenFlex MID360 network settings only.
  --env              Install OpenFlex runtime/build environment only.
  --environment      Install OpenFlex runtime/build environment only.
  --compile          Build OpenFlex workspace packages only.
  --kcan             Install KCAN Linux driver only.
  --sync-source      Check per-component metadata and download/update sources only.
  --dry-run          Print install/build steps without executing them.
  --skip-install     Do not install deb packages from openflex_drivers.
  --skip-build       Do not build workspace packages.
  --skip-desktop     Do not create the OpenFlex desktop launcher.
  --jobs N           Pass --parallel-workers N to colcon. Default: 2.
  -h, --help         Show this help.

The source sync mode reads openflex_component.yaml from each managed repository,
compares its revision with the same Branch on GitHub, and asks before downloading
or updating a component. Local .git metadata is not required.

The script installs all .deb packages found in:
  ../openflex_drivers/*.deb

Then it builds selected packages one step at a time with colcon. Dependencies
that must be available before later packages are built first, and each step
sources install/setup.bash before continuing.

VR software install:
  Installs adb, then installs the selected Pico or Quest APK from:
  ../openflex_vr_apk/apk/
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --openflex)
      MODE="openflex"
      shift
      ;;
    --desktop)
      MODE="desktop"
      shift
      ;;
    --vr)
      MODE="vr"
      shift
      ;;
    --camera)
      MODE="camera"
      shift
      ;;
    --lidar)
      MODE="lidar"
      shift
      ;;
    --env|--environment)
      MODE="environment"
      shift
      ;;
    --compile)
      MODE="compile"
      shift
      ;;
    --kcan)
      MODE="kcan"
      shift
      ;;
    --sync-source)
      MODE="sync-source"
      shift
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --skip-install)
      SKIP_INSTALL=1
      shift
      ;;
    --skip-build)
      SKIP_BUILD=1
      shift
      ;;
    --skip-desktop)
      SKIP_DESKTOP=1
      shift
      ;;
    --jobs)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --jobs requires a value" >&2
        exit 2
      fi
      PARALLEL_WORKERS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

run() {
  echo "+ $*"
  if [[ "${DRY_RUN}" -eq 0 ]]; then
    "$@"
  fi
}

require_not_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    echo "ERROR: do not run this script with sudo." >&2
    echo "       The script will ask for sudo only when installing deb dependencies." >&2
    echo "       ROS/colcon packages must be built as the normal user." >&2
    exit 1
  fi
}

request_sudo_for_install() {
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo -v"
    return
  fi

  echo "System deb installation requires sudo. Please enter your password if prompted."
  sudo -v
}

ensure_git_available() {
  if command -v git >/dev/null 2>&1; then
    return
  fi

  echo "git is required to download OpenFlex source repositories."
  request_sudo_for_install

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo apt update -y"
    echo "+ sudo apt-get install -y git"
    return
  fi

  run sudo apt update -y
  run sudo apt-get install -y git
}

source_if_exists() {
  local setup_file="$1"
  if [[ -f "${setup_file}" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${setup_file}"
    set -u
  fi
}

source_required() {
  local setup_file="$1"
  require_file "${setup_file}"
  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

require_sync_tools() {
  local command_name
  for command_name in curl unzip; do
    if ! command -v "${command_name}" >/dev/null 2>&1; then
      echo "ERROR: ${command_name} is required for source download/update." >&2
      exit 1
    fi
  done
}

manifest_value() {
  local manifest="$1"
  local key="$2"
  awk -F: -v wanted="${key}" '$1 ~ "^[[:space:]]*" wanted "[[:space:]]*$" {sub(/^[[:space:]]*/, "", $2); sub(/[[:space:]]*$/, "", $2); print $2; exit}' "${manifest}"
}

manifest_required_files() {
  local manifest="$1"
  awk '
    /^required_files:[[:space:]]*$/ { in_list=1; next }
    in_list && /^  -[[:space:]]+/ { sub(/^  -[[:space:]]+/, ""); print; next }
    in_list && !/^[[:space:]]*$/ { exit }
  ' "${manifest}"
}

revision_is_newer() {
  local local_revision="$1"
  local remote_revision="$2"
  [[ "${local_revision}" != "${remote_revision}" && "$(printf '%s\n%s\n' "${local_revision}" "${remote_revision}" | sort -V | tail -n1)" == "${remote_revision}" ]]
}

component_is_complete() {
  local target_path="$1"
  local component_name="$2"
  local expected_branch="$3"
  local manifest="${target_path}/openflex_component.yaml"
  local required_file

  [[ -d "${target_path}" && -f "${manifest}" ]] || return 1
  [[ "$(manifest_value "${manifest}" name)" == "${component_name}" ]] || return 1
  [[ "$(manifest_value "${manifest}" branch)" == "${expected_branch}" ]] || return 1

  while IFS= read -r required_file; do
    [[ -z "${required_file}" ]] && continue
    [[ -e "${target_path}/${required_file}" ]] || return 1
  done < <(manifest_required_files "${manifest}")
}

confirm_sync_action() {
  local prompt="$1"
  if [[ "${DRY_RUN}" -eq 1 || ! -t 0 ]]; then
    echo "  ${prompt} [dry-run/non-interactive: no]"
    return 1
  fi
  local answer
  read -r -p "  ${prompt} [y/N]: " answer
  [[ "${answer,,}" == "y" || "${answer,,}" == "yes" ]]
}

require_file() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: missing required file: ${path}" >&2
    exit 1
  fi
}

require_dir() {
  local path="$1"
  if [[ ! -d "${path}" ]]; then
    echo "ERROR: missing required directory: ${path}" >&2
    exit 1
  fi
}

trim_whitespace() {
  local value="$1"
  value="$(printf '%s' "${value}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
  printf '%s' "${value}"
}

first_existing_file() {
  local candidate
  for candidate in "$@"; do
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

# swerve_base_panel is already tracked inside system_bringup_layer. Cloning it
# separately would conflict with the non-empty directory on a fresh install.
openflex_repo_specs() {
  cat <<'EOF'
openflex_drivers|openflex_drivers
openflex_vr_apk|openflex_vr_apk
OpenFleX|openflex_ws/src/OpenFleX
openflex_EXO|openflex_ws/src/openflex_EXO
openflex_mujoco|openflex_ws/src/openflex_mujoco
openflex_vr_bridge|openflex_ws/src/openflex_vr_bridge
openflex_vla|openflex_ws/src/openflex_vla
openarmx_description|openflex_ws/src/openflex_armx/openarmx_description
openarmx_ros2|openflex_ws/src/openflex_armx/openarmx_ros2
openarmx_teleop_vr|openflex_ws/src/openflex_armx/openarmx_teleop_vr
openarmx_tools|openflex_ws/src/openflex_armx/openarmx_tools
openarmx_hands|openflex_ws/src/openflex_armx/openarmx_hands
base_model_interface_layer|openflex_ws/src/openflex_chassis/base_model_interface_layer
hardware_sensor_layer|openflex_ws/src/openflex_chassis/hardware_sensor_layer
mapping_localization_layer|openflex_ws/src/openflex_chassis/mapping_localization_layer
motion_control_layer|openflex_ws/src/openflex_chassis/motion_control_layer
navigation_layer|openflex_ws/src/openflex_chassis/navigation_layer
system_bringup_layer|openflex_ws/src/openflex_chassis/system_bringup_layer
tools_common_layer|openflex_ws/src/openflex_chassis/tools_common_layer
openarmx_head_bringup|openflex_ws/src/openflex_head/openarmx_head_bringup
openarmx_head_description|openflex_ws/src/openflex_head/openarmx_head_description
openarmx_head_hardware|openflex_ws/src/openflex_head/openarmx_head_hardware
openarmx_head_teleop_vr_pico|openflex_ws/src/openflex_head/openarmx_head_teleop_vr_pico
openarmx_head_tools|openflex_ws/src/openflex_head/openarmx_head_tools
openarmx_head_visio_h264|openflex_ws/src/openflex_head/openarmx_head_visio_h264
openarmx_integrated_bringup|openflex_ws/src/openflex_integrated/openarmx_integrated_bringup
openarmx_integrated_description|openflex_ws/src/openflex_integrated/openarmx_integrated_description
openflex_gui|openflex_ws/src/openflex_integrated/openflex_gui
openflex_manager|openflex_ws/src/openflex_integrated/openflex_manager
lift_slide_bringup|openflex_ws/src/openflex_lift_slide/lift_slide_bringup
lift_slide_description|openflex_ws/src/openflex_lift_slide/lift_slide_description
lift_slide_driver|openflex_ws/src/openflex_lift_slide/lift_slide_driver
lift_slide_msgs|openflex_ws/src/openflex_lift_slide/lift_slide_msgs
lift_slide_panel|openflex_ws/src/openflex_lift_slide/lift_slide_panel
EOF
}

refresh_workspace_paths() {
  WORKSPACE_DIR="${BOOTSTRAP_ROOT}/openflex_ws"
  OPENFLEX_ROOT="${BOOTSTRAP_ROOT}"
  DRIVER_DIR="${OPENFLEX_ROOT}/openflex_drivers"
}

ensure_bootstrap_layout() {
  refresh_workspace_paths
  echo "Preparing OpenFlex workspace layout:"
  echo "  ${OPENFLEX_ROOT}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ mkdir -p ${OPENFLEX_ROOT}/openflex_maps ${OPENFLEX_ROOT}/openflex_models ${WORKSPACE_DIR}/src"
    echo "+ mkdir -p ${WORKSPACE_DIR}/src/openflex_armx ${WORKSPACE_DIR}/src/openflex_head ${WORKSPACE_DIR}/src/openflex_integrated ${WORKSPACE_DIR}/src/openflex_lift_slide ${WORKSPACE_DIR}/src/openflex_chassis"
    return
  fi

  mkdir -p \
    "${OPENFLEX_ROOT}/openflex_maps" \
    "${OPENFLEX_ROOT}/openflex_models" \
    "${WORKSPACE_DIR}/src" \
    "${WORKSPACE_DIR}/src/openflex_armx" \
    "${WORKSPACE_DIR}/src/openflex_head" \
    "${WORKSPACE_DIR}/src/openflex_integrated" \
    "${WORKSPACE_DIR}/src/openflex_lift_slide" \
    "${WORKSPACE_DIR}/src/openflex_chassis"
}

sync_component() {
  local repo_name="$1"
  local relative_path="$2"
  local target_path="${OPENFLEX_ROOT}/${relative_path}"
  local remote_manifest_url="https://raw.githubusercontent.com/${OPENFLEX_GIT_ORG##*/}/${repo_name}/${OPENFLEX_GIT_BRANCH}/openflex_component.yaml"
  local archive_url="https://codeload.github.com/${OPENFLEX_GIT_ORG##*/}/${repo_name}/zip/refs/heads/${OPENFLEX_GIT_BRANCH}"
  local local_manifest="${target_path}/openflex_component.yaml"
  local remote_manifest
  local remote_name
  local remote_branch
  local update_policy="check_and_update"
  local local_revision="0"
  local remote_revision="0"
  local temp_dir archive_file extracted_dir backup_path

  echo
  echo "Checking component: ${repo_name}"
  echo "  path:   ${target_path}"
  echo "  branch: ${OPENFLEX_GIT_BRANCH}"

  if [[ -f "${local_manifest}" ]]; then
    local_revision="$(manifest_value "${local_manifest}" revision || true)"
    [[ -n "${local_revision}" ]] || local_revision="0"
    update_policy="$(manifest_value "${local_manifest}" update_policy || true)"
    [[ -n "${update_policy}" ]] || update_policy="check_and_update"
  fi

  # Some components, such as VLA, are intentionally installed once and then
  # managed outside this installer. A complete local copy must not trigger a
  # remote metadata request or an update prompt for that policy.
  if [[ "${update_policy}" == "download_if_missing" ]] && component_is_complete "${target_path}" "${repo_name}" "${OPENFLEX_GIT_BRANCH}"; then
    echo "  local component exists; update policy is download_if_missing, skipping."
    return 0
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ curl -fsSL ${remote_manifest_url}"
    echo "+ compare local revision ${local_revision} with remote revision"
    if ! component_is_complete "${target_path}" "${repo_name}" "${OPENFLEX_GIT_BRANCH}"; then
      echo "  local component is missing or incomplete; would ask before downloading."
    else
      echo "  local component is structurally complete; would ask only if remote revision is newer."
    fi
    return
  fi

  require_sync_tools
  remote_manifest="$(curl -fsSL --retry 2 --connect-timeout 10 "${remote_manifest_url}")" || {
    echo "ERROR: failed to read remote component metadata: ${remote_manifest_url}" >&2
    return 1
  }
  remote_revision="$(printf '%s\n' "${remote_manifest}" | awk -F: '$1 ~ /^[[:space:]]*revision[[:space:]]*$/ {sub(/^[[:space:]]*/, "", $2); sub(/[[:space:]]*$/, "", $2); print $2; exit}')"
  if [[ -z "${remote_revision}" ]]; then
    echo "ERROR: remote metadata has no revision: ${repo_name}" >&2
    return 1
  fi
  remote_name="$(printf '%s\n' "${remote_manifest}" | awk -F: '$1 ~ /^[[:space:]]*name[[:space:]]*$/ {sub(/^[[:space:]]*/, "", $2); sub(/[[:space:]]*$/, "", $2); print $2; exit}')"
  remote_branch="$(printf '%s\n' "${remote_manifest}" | awk -F: '$1 ~ /^[[:space:]]*branch[[:space:]]*$/ {sub(/^[[:space:]]*/, "", $2); sub(/[[:space:]]*$/, "", $2); print $2; exit}')"
  if [[ "${remote_name}" != "${repo_name}" || "${remote_branch}" != "${OPENFLEX_GIT_BRANCH}" ]]; then
    echo "ERROR: remote metadata identity mismatch for ${repo_name}: name=${remote_name}, branch=${remote_branch}" >&2
    return 1
  fi

  if ! component_is_complete "${target_path}" "${repo_name}" "${OPENFLEX_GIT_BRANCH}"; then
    echo "  local component is missing or incomplete."
    if ! confirm_sync_action "Download ${repo_name} revision ${remote_revision}?"; then
      echo "  skipped."
      return 0
    fi
  elif revision_is_newer "${local_revision}" "${remote_revision}"; then
    echo "  update available: local ${local_revision}, remote ${remote_revision}"
    if ! confirm_sync_action "Update ${repo_name} to revision ${remote_revision}?"; then
      echo "  kept local revision ${local_revision}."
      return 0
    fi
  else
    echo "  up to date at revision ${local_revision}."
    return 0
  fi

  target_real="$(cd "${SCRIPT_DIR}" && pwd)"
  if [[ -d "${target_path}" && "$(cd "${target_path}" && pwd)" == "${target_real}" ]]; then
    echo "  skipping self-update while the installer is running."
    return 0
  fi

  temp_dir="$(mktemp -d)"
  archive_file="${temp_dir}/${repo_name}.zip"
  if ! curl -fL --retry 2 --connect-timeout 10 -o "${archive_file}" "${archive_url}"; then
    echo "ERROR: failed to download ${repo_name} from ${archive_url}" >&2
    return 1
  fi
  if ! unzip -q "${archive_file}" -d "${temp_dir}"; then
    echo "ERROR: downloaded archive is invalid: ${repo_name}" >&2
    return 1
  fi
  extracted_dir="$(find "${temp_dir}" -mindepth 1 -maxdepth 1 -type d ! -name '.*' -print -quit)"
  if [[ -z "${extracted_dir}" ]] || ! component_is_complete "${extracted_dir}" "${repo_name}" "${OPENFLEX_GIT_BRANCH}"; then
    echo "ERROR: downloaded component failed completeness check: ${repo_name}" >&2
    return 1
  fi

  mkdir -p "$(dirname "${target_path}")"
  if [[ -e "${target_path}" ]]; then
    backup_path="${target_path}.backup.$(date +%Y%m%d-%H%M%S)"
    mv "${target_path}" "${backup_path}"
    echo "  backed up previous component to ${backup_path}"
  fi
  mv "${extracted_dir}" "${target_path}"
  echo "  installed ${repo_name} revision ${remote_revision}."
}

prepare_openflex_sources() {
  require_not_root
  require_sync_tools

  ensure_bootstrap_layout
  echo "Using OpenFlex git source:"
  echo "  org:    ${OPENFLEX_GIT_ORG}"
  echo "  branch: ${OPENFLEX_GIT_BRANCH}"

  local spec repo_name relative_path
  while IFS= read -r spec; do
    [[ -z "${spec}" ]] && continue
    repo_name="${spec%%|*}"
    relative_path="${spec#*|}"
    sync_component "${repo_name}" "${relative_path}"
  done < <(openflex_repo_specs)
}

choose_mode() {
  if [[ -n "${MODE}" ]]; then
    return
  fi

  cat <<'MENU'
Please choose an installation option:
  1. 初次安装 OpenFlex
  2. 安装桌面 GUI
  3. 安装 VR 软件
  4. 配置相机
  5. 配置雷达
  6. 仅下载/更新源码
  7. 环境安装
  8. 编译
  9. 安装 KCAN 驱动
MENU

  local choice
  read -r -p "请输入选项 [1/2/3/4/5/6/7/8/9]: " choice
  case "${choice}" in
    1)
      MODE="openflex"
      ;;
    2)
      MODE="desktop"
      ;;
    3)
      MODE="vr"
      ;;
    4)
      MODE="camera"
      ;;
    5)
      MODE="lidar"
      ;;
    6)
      MODE="sync-source"
      ;;
    7)
      MODE="environment"
      ;;
    8)
      MODE="compile"
      ;;
    9)
      MODE="kcan"
      ;;
    *)
      echo "ERROR: invalid option: ${choice}" >&2
      exit 2
      ;;
  esac
}

ensure_workspace() {
  require_dir "${WORKSPACE_DIR}/src"
}

ensure_user_camera_config() {
  local user_config_dir="${HOME}/.openflex"
  local user_camera_config="${user_config_dir}/cameras_config.yaml"

  if [[ -f "${user_camera_config}" ]]; then
    echo "Using existing OpenFlex camera config:"
    echo "  ${user_camera_config}"
    return
  fi

  echo "Creating OpenFlex user camera config:"
  echo "  ${user_camera_config}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ mkdir -p ${user_config_dir}"
    echo "+ generate default ${user_camera_config}"
    return
  fi

  mkdir -p "${user_config_dir}"
  cat > "${user_camera_config}" <<'EOF'
cameras:
  right_wrist:
    serial_no: ""
  left_wrist:
    serial_no: ""
  head:
    serial_no: ""
  base:
    serial_no: ""
EOF
}

sync_camera_serials() {
  local right_wrist_serial="$1"
  local left_wrist_serial="$2"
  local head_serial="$3"
  local base_serial="$4"
  shift 4
  local camera_configs=("$@")
  local config_path

  for config_path in "${camera_configs[@]}"; do
    require_file "${config_path}"
  done

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    for config_path in "${camera_configs[@]}"; do
      echo "+ update selected camera serial_no values in ${config_path}"
    done
    return
  fi

  python3 - \
    "${right_wrist_serial}" \
    "${left_wrist_serial}" \
    "${head_serial}" \
    "${base_serial}" \
    "${camera_configs[@]}" <<'PYTHON_EOF'
import re
import sys
from pathlib import Path


serials = dict(zip(
    ("right_wrist", "left_wrist", "head", "base"),
    sys.argv[1:5],
))
paths = [Path(raw_path) for raw_path in sys.argv[5:]]
invalid_serials = [
    camera_key
    for camera_key, serial in serials.items()
    if serial and re.fullmatch(r"[A-Za-z0-9_-]+", serial) is None
]
if invalid_serials:
    raise SystemExit(
        "ERROR: invalid camera serial for: " + ", ".join(invalid_serials)
    )


def indent_width(line: str) -> int:
    return len(line) - len(line.lstrip())


def update_config(path: Path) -> str:
    original = path.read_text(encoding="utf-8")
    lines = original.splitlines(keepends=True)
    cameras_index = None
    cameras_indent = None

    for index, line in enumerate(lines):
        content = line.rstrip("\r\n")
        match = re.fullmatch(r"(\s*)cameras:\s*(?:#.*)?", content)
        if match:
            cameras_index = index
            cameras_indent = len(match.group(1))
            break
    if cameras_index is None or cameras_indent is None:
        raise ValueError(f"missing cameras mapping: {path}")

    replacements: dict[int, str] = {}
    for camera_key, serial in serials.items():
        key_index = None
        key_indent = None
        key_pattern = re.compile(rf"(\s*){re.escape(camera_key)}:\s*(?:#.*)?")
        for index in range(cameras_index + 1, len(lines)):
            content = lines[index].rstrip("\r\n")
            stripped = content.strip()
            if stripped and not stripped.startswith("#") and indent_width(content) <= cameras_indent:
                break
            match = key_pattern.fullmatch(content)
            if match and len(match.group(1)) > cameras_indent:
                key_index = index
                key_indent = len(match.group(1))
                break
        if key_index is None or key_indent is None:
            raise ValueError(f"missing cameras.{camera_key}: {path}")

        serial_index = None
        serial_match = None
        for index in range(key_index + 1, len(lines)):
            content = lines[index].rstrip("\r\n")
            stripped = content.strip()
            if stripped and not stripped.startswith("#") and indent_width(content) <= key_indent:
                break
            match = re.fullmatch(
                r'(\s*serial_no:\s*)"[^"]*"(\s*(?:#.*)?)',
                content,
            )
            if match and indent_width(content) > key_indent:
                serial_index = index
                serial_match = match
                break
        if serial_index is None or serial_match is None:
            raise ValueError(f"missing quoted cameras.{camera_key}.serial_no: {path}")

        if serial:
            newline = lines[serial_index][len(lines[serial_index].rstrip("\r\n")):]
            replacements[serial_index] = (
                f'{serial_match.group(1)}"{serial}"{serial_match.group(2)}{newline}'
            )

    for index, replacement in replacements.items():
        lines[index] = replacement
    return "".join(lines)


try:
    updated_configs = [(path, update_config(path)) for path in paths]
except (OSError, ValueError) as exc:
    raise SystemExit(f"ERROR: invalid camera config: {exc}") from exc

for path, updated_text in updated_configs:
    path.write_text(updated_text, encoding="utf-8")
PYTHON_EOF
}

configure_cameras() {
  require_not_root
  local user_config_dir="${HOME}/.openflex"
  local user_camera_config="${user_config_dir}/cameras_config.yaml"
  local package_camera_config_dir="${WORKSPACE_DIR}/src/openflex_vla/config/cameras"
  local package_camera_config="${package_camera_config_dir}/cameras_config.yaml"
  local package_camera_config_30fps="${package_camera_config_dir}/cameras_config_30fps.yaml"
  local camera_configs=("${user_camera_config}")

  # 确保配置文件存在
  if [[ ! -f "${user_camera_config}" ]]; then
    echo "相机配置文件不存在，正在创建..."
    ensure_user_camera_config
  fi

  if [[ ! -f "${user_camera_config}" ]]; then
    if [[ "${DRY_RUN}" -eq 1 ]]; then
      echo "Dry run: camera config was not created, skipping interactive updates."
      return 0
    fi
    echo "ERROR: failed to create camera config: ${user_camera_config}" >&2
    exit 1
  fi

  # 工作区配置是可选同步目标：文件存在就同步，不存在则跳过。
  # 用户配置始终保留在同步列表中，确保部署中心至少能更新 ~/.openflex 配置。
  for optional_config in "${package_camera_config}" "${package_camera_config_30fps}"; do
    if [[ -f "${optional_config}" ]]; then
      camera_configs+=("${optional_config}")
    else
      echo "跳过不存在的相机配置文件: ${optional_config}"
    fi
  done

  echo ""
  echo "═══════════════════════════════════════════════════════"
  echo "           OpenFlex 相机配置工具"
  echo "═══════════════════════════════════════════════════════"
  echo ""
  echo "配置文件位置: ${user_camera_config}"
  echo ""
  echo "提示："
  echo "  - 直接按 Enter 或输入空格后按 Enter 跳过该相机配置"
  echo "  - 输入序列号后按 Enter 确认"
  echo "  - 使用 'rs-enumerate-devices -s' 命令查看连接的相机"
  echo ""
  echo "───────────────────────────────────────────────────────"

  # 读取当前配置
  local right_wrist_sn=""
  local left_wrist_sn=""
  local head_sn=""
  local base_sn=""

  if command -v yq &>/dev/null; then
    right_wrist_sn=$(yq e '.cameras.right_wrist.serial_no' "${user_camera_config}" 2>/dev/null | tr -d '"')
    left_wrist_sn=$(yq e '.cameras.left_wrist.serial_no' "${user_camera_config}" 2>/dev/null | tr -d '"')
    head_sn=$(yq e '.cameras.head.serial_no' "${user_camera_config}" 2>/dev/null | tr -d '"')
    base_sn=$(yq e '.cameras.base.serial_no' "${user_camera_config}" 2>/dev/null | tr -d '"')
  else
    right_wrist_sn=$(grep -A2 "right_wrist:" "${user_camera_config}" | grep "serial_no:" | sed 's/.*"\(.*\)".*/\1/' || echo "")
    left_wrist_sn=$(grep -A2 "left_wrist:" "${user_camera_config}" | grep "serial_no:" | sed 's/.*"\(.*\)".*/\1/' || echo "")
    head_sn=$(grep -A2 "head:" "${user_camera_config}" | grep "serial_no:" | sed 's/.*"\(.*\)".*/\1/' || echo "")
    base_sn=$(grep -A2 "base:" "${user_camera_config}" | grep "serial_no:" | sed 's/.*"\(.*\)".*/\1/' || echo "")
  fi

  # 交互式输入
  local new_right_wrist=""
  local new_left_wrist=""
  local new_head=""
  local new_base=""

  echo ""
  read -r -p "右腕相机 ID [当前: ${right_wrist_sn}]: " new_right_wrist
  read -r -p "左腕相机 ID [当前: ${left_wrist_sn}]: " new_left_wrist
  read -r -p "头部相机 ID [当前: ${head_sn}]: " new_head
  read -r -p "底盘相机 ID [当前: ${base_sn}]: " new_base

  new_right_wrist="$(trim_whitespace "${new_right_wrist}")"
  new_left_wrist="$(trim_whitespace "${new_left_wrist}")"
  new_head="$(trim_whitespace "${new_head}")"
  new_base="$(trim_whitespace "${new_base}")"

  # 同步存在的配置文件；空输入保持对应相机原值不变。
  local updated=0
  if [[ -n "${new_right_wrist}" || -n "${new_left_wrist}" || -n "${new_head}" || -n "${new_base}" ]]; then
    sync_camera_serials \
      "${new_right_wrist}" \
      "${new_left_wrist}" \
      "${new_head}" \
      "${new_base}" \
      "${camera_configs[@]}"
    updated=1
  fi

  echo ""
  echo "───────────────────────────────────────────────────────"
  if [[ "${updated}" -eq 1 ]]; then
    echo "✓ 相机配置已更新"
    echo ""
    echo "更新后的配置："
    echo ""
    grep -A2 "right_wrist:\|left_wrist:\|head:\|base:" "${user_camera_config}" | grep -E "(right_wrist|left_wrist|head|base|serial_no)"
    echo ""
    echo "已同步配置文件："
    printf '  %s\n' "${camera_configs[@]}"
  else
    echo "未做任何修改"
  fi
  echo ""
  echo "配置文件位置: ${user_camera_config}"
  echo "═══════════════════════════════════════════════════════"
  echo ""
}

ensure_user_lidar_config() {
  local user_config_dir="${HOME}/.openflex"
  local user_lidar_config="${user_config_dir}/lidar_config.yaml"
  local default_lidar_config=""
  local mid360_json="${WORKSPACE_DIR}/src/openflex_chassis/hardware_sensor_layer/livox_ros_driver2/config/MID360_config.json"
  default_lidar_config="$(first_existing_file || true)"

  if [[ -f "${user_lidar_config}" ]]; then
    echo "Using existing OpenFlex lidar config:"
    echo "  ${user_lidar_config}"
    return
  fi

  echo "Creating OpenFlex user lidar config:"
  echo "  ${user_lidar_config}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ mkdir -p ${user_config_dir}"
    if [[ -f "${default_lidar_config}" ]]; then
      echo "+ cp ${default_lidar_config} ${user_lidar_config}"
    else
      echo "+ generate ${user_lidar_config} from ${mid360_json}"
    fi
    return
  fi

  mkdir -p "${user_config_dir}"
  if [[ -f "${default_lidar_config}" ]]; then
    cp "${default_lidar_config}" "${user_lidar_config}"
    return
  fi

  require_file "${mid360_json}"
  python3 - "${mid360_json}" "${user_lidar_config}" <<'PYTHON_EOF'
import json
import sys
from pathlib import Path

mid360_json = Path(sys.argv[1])
user_lidar_config = Path(sys.argv[2])

with mid360_json.open("r", encoding="utf-8") as f:
    config = json.load(f)

mid360_section = config.get("Mid360s") or config.get("MID360") or {}
host_info = mid360_section.get("host_net_info", {})
host_ip = (
    host_info.get("point_data_ip")
    or host_info.get("cmd_data_ip")
    or host_info.get("imu_data_ip")
    or "192.168.1.50"
)
lidar_configs = config.get("lidar_configs", [])
lidar_ip = "192.168.1.173"
if lidar_configs and isinstance(lidar_configs[0], dict):
    lidar_ip = lidar_configs[0].get("ip") or lidar_ip

user_lidar_config.write_text(
    "lidar_config_name: openflex_default_lidar\n"
    "version: 1\n"
    "\n"
    "host_network:\n"
    f"  ip: \"{host_ip}\"\n"
    "\n"
    "lidars:\n"
    "  mid360:\n"
    f"    device_ip: \"{lidar_ip}\"\n",
    encoding="utf-8",
)
PYTHON_EOF
}

configure_lidar() {
  require_not_root
  local user_config_dir="${HOME}/.openflex"
  local user_lidar_config="${user_config_dir}/lidar_config.yaml"
  local mid360_json="${WORKSPACE_DIR}/src/openflex_chassis/hardware_sensor_layer/livox_ros_driver2/config/MID360_config.json"

  # 确保配置文件存在
  if [[ ! -f "${user_lidar_config}" ]]; then
    echo "雷达配置文件不存在，正在创建..."
    ensure_user_lidar_config
  fi

  if [[ ! -f "${user_lidar_config}" ]]; then
    if [[ "${DRY_RUN}" -eq 1 ]]; then
      echo "Dry run: lidar config was not created, skipping interactive updates."
      return 0
    fi
    echo "ERROR: failed to create lidar config: ${user_lidar_config}" >&2
    exit 1
  fi

  echo ""
  echo "═══════════════════════════════════════════════════════"
  echo "           OpenFlex 雷达网络配置向导"
  echo "═══════════════════════════════════════════════════════"
  echo ""
  echo "【步骤 1：配置电脑网口 IP】"
  echo ""
  echo "  请在系统设置中找到连接雷达的网口，配置 IP 为："
  echo "  格式：192.168.1.XX（XX 为任意两位数字，建议 50）"
  echo ""
  echo "  操作步骤："
  echo "    1. 打开 [设置] → [网络]"
  echo "    2. 选择连接雷达的网口（如 eno1 或 eth0）"
  echo "    3. 点击设置图标 → [IPv4] → 方式改为 [手动]"
  echo "    4. 地址：192.168.1.XX"
  echo "    5. 子网掩码：255.255.255.0"
  echo "    6. 点击 [应用] 并重新连接"
  echo ""
  read -r -p "  完成网口配置后，按 Enter 继续..."

  echo ""
  echo "───────────────────────────────────────────────────────"

  # 读取当前配置
  local host_ip=""
  local lidar_ip=""

  if command -v yq &>/dev/null; then
    host_ip=$(yq e '.host_network.ip' "${user_lidar_config}" 2>/dev/null | tr -d '"')
    lidar_ip=$(yq e '.lidars.mid360.device_ip' "${user_lidar_config}" 2>/dev/null | tr -d '"')
  else
    host_ip=$(grep -A1 "host_network:" "${user_lidar_config}" | grep "ip:" | sed 's/.*"\(.*\)".*/\1/' || echo "192.168.1.50")
    lidar_ip=$(grep -A3 "mid360:" "${user_lidar_config}" | grep "device_ip:" | sed 's/.*"\(.*\)".*/\1/' || echo "192.168.1.173")
  fi

  # 提取主机末段和雷达 SN 后两位。MID360 设备 IP 规则是
  # 192.168.1.1XX，其中 XX 来自雷达 SN 最后两位。
  local host_last=$(echo "${host_ip}" | awk -F. '{print $4}')
  local lidar_last=$(echo "${lidar_ip}" | awk -F. '{print $4}')
  local lidar_sn_suffix="${lidar_last}"
  if [[ "${lidar_last}" =~ ^1([0-9]{2})$ ]]; then
    lidar_sn_suffix="${BASH_REMATCH[1]}"
  fi

  echo ""
  echo "【配置主机网口 IP 最后两位】"
  echo ""
  echo "  当前完整 IP：192.168.1.${host_last}"
  echo ""
  read -r -p "  请输入主机网口 IP 最后两位数字 [直接回车保持 ${host_last}]: " new_host_last

  echo ""
  echo "───────────────────────────────────────────────────────"
  echo ""
  echo "【步骤 2：配置雷达设备 IP】"
  echo ""
  echo "  雷达 IP 格式：192.168.1.1XX（最后两位）"
  echo "  提示：请找到雷达机身标签上的序列号（SN）最后两位"
  echo "  示例：SN 末尾为 73 → IP 为 192.168.1.173"
  echo ""
  echo "  当前完整 IP：${lidar_ip}"
  echo ""
  read -r -p "  请输入雷达 SN 号最后两位数字 [直接回车保持 ${lidar_sn_suffix}]: " new_lidar_last

  # 构建完整IP并更新
  local updated=0

  if [[ -n "${new_host_last}" ]]; then
    if [[ "${new_host_last}" =~ ^[0-9]+$ ]]; then
      local new_host_ip="192.168.1.${new_host_last}"
      if [[ "${DRY_RUN}" -eq 1 ]]; then
        echo "+ update host IP in ${user_lidar_config}: ${new_host_ip}"
      else
        sed -i "/host_network:/,/ip:/ s|ip: \".*\"|ip: \"${new_host_ip}\"|" "${user_lidar_config}"
      fi
      echo ""
      echo "  ✓ 已更新主机 IP：192.168.1.${new_host_last}"
      host_ip="${new_host_ip}"
      updated=1
    else
      echo ""
      echo "  ✗ 输入无效（需要数字），保持原值"
    fi
  fi

  if [[ -n "${new_lidar_last}" ]]; then
    if [[ "${new_lidar_last}" =~ ^[0-9]{1,2}$ ]]; then
      local new_lidar_suffix
      printf -v new_lidar_suffix "%02d" "${new_lidar_last}"
      local new_lidar_ip="192.168.1.1${new_lidar_suffix}"
      if [[ "${DRY_RUN}" -eq 1 ]]; then
        echo "+ update lidar IP in ${user_lidar_config}: ${new_lidar_ip}"
      else
        sed -i "/mid360:/,/device_ip:/ s|device_ip: \".*\"|device_ip: \"${new_lidar_ip}\"|" "${user_lidar_config}"
      fi
      echo "  ✓ 已更新雷达 IP：${new_lidar_ip}"
      lidar_ip="${new_lidar_ip}"
      updated=1
    else
      echo "  ✗ 输入无效（需要 SN 最后两位数字），保持原值"
    fi
  fi

  echo ""
  echo "═══════════════════════════════════════════════════════"

  if [[ "${updated}" -eq 1 ]]; then
    echo "✓ 雷达配置已更新"

    # 同步更新 MID360_config.json
    if [[ -f "${mid360_json}" ]]; then
      echo ""
      echo "正在同步更新驱动配置文件..."

      if [[ "${DRY_RUN}" -eq 1 ]]; then
        echo "+ update ${mid360_json}: host_ip=${host_ip}, lidar_ip=${lidar_ip}"
      else
      python3 << PYTHON_EOF
import json
import sys

json_file = "${mid360_json}"
host_ip = "${host_ip}"
lidar_ip = "${lidar_ip}"

try:
    with open(json_file, 'r') as f:
        config = json.load(f)

    # 更新主机IP（4个地方），兼容旧配置中的 Mid360s 和当前配置中的 MID360。
    mid360_section = config.get('Mid360s') or config.get('MID360')
    if isinstance(mid360_section, dict) and 'host_net_info' in mid360_section:
        mid360_section['host_net_info']['cmd_data_ip'] = host_ip
        mid360_section['host_net_info']['point_data_ip'] = host_ip
        mid360_section['host_net_info']['imu_data_ip'] = host_ip
        mid360_section['host_net_info']['push_msg_ip'] = host_ip

    # 更新雷达IP
    if 'lidar_configs' in config and len(config['lidar_configs']) > 0:
        config['lidar_configs'][0]['ip'] = lidar_ip

    with open(json_file, 'w') as f:
        json.dump(config, f, indent=2)

    print("  ✓ MID360_config.json 已同步更新")
except Exception as e:
    print(f"  ✗ 更新失败: {e}", file=sys.stderr)
    sys.exit(1)
PYTHON_EOF
      fi

      if [[ $? -eq 0 ]]; then
        echo ""
        echo "【配置完成】"
        echo "  • 主机网口 IP：${host_ip}"
        echo "  • 雷达设备 IP：${lidar_ip}"
        echo ""
        echo "【验证连接】"
        echo "  测试命令：ping ${lidar_ip}"
        echo ""
        echo "【启动雷达】"
        echo "  启动命令：ros2 launch livox_ros_driver2 msg_MID360.launch.py"
      fi
    else
      echo "  ⚠ 警告：驱动配置文件未找到"
    fi
  else
    echo "未做任何修改"
  fi
  
  echo ""
  echo "配置文件位置:"
  echo "  用户配置: ${user_lidar_config}"
  echo "  驱动配置: ${mid360_json}"
  echo "═══════════════════════════════════════════════════════"
  echo ""
}


install_driver_debs() {
  local deb_files=()
  mapfile -t deb_files < <(find "${DRIVER_DIR}" -maxdepth 1 -type f -name "*.deb" | sort)

  if [[ "${#deb_files[@]}" -eq 0 ]]; then
    echo "ERROR: no driver deb packages found in: ${DRIVER_DIR}" >&2
    exit 1
  fi

  echo "Installing driver debs:"
  printf '  %s\n' "${deb_files[@]}"
  request_sudo_for_install
  run sudo apt install --allow-downgrades --reinstall -y "${deb_files[@]}"
  run sudo ldconfig
}

install_camera_dependencies() {
  echo "Installing camera dependencies:"
  echo "  RealSense SDK and ROS camera packages"

  request_sudo_for_install

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ disable stale /etc/apt/sources.list.d/librealsense*.list entries"
    echo "+ sudo apt update"
    echo "+ sudo apt install -y curl gnupg apt-transport-https lsb-release v4l-utils"
    echo "+ sudo mkdir -p /etc/apt/keyrings"
    echo "+ curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null"
    echo "+ write /etc/apt/sources.list.d/librealsense.list"
    echo "+ sudo apt update"
    echo "+ sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-udev-rules ros-humble-realsense2-camera ros-humble-realsense2-description ros-humble-usb-cam"
    return
  fi

  # A stale RealSense source can make the prerequisite apt update fail before this
  # function gets a chance to refresh its signing key. Rebuild the source below.
  local source_file
  for source_file in /etc/apt/sources.list.d/librealsense*.list; do
    [[ -e "${source_file}" ]] || continue
    run sudo mv "${source_file}" "${source_file}.disabled-by-openflex"
  done

  run sudo apt update
  run sudo apt install -y curl gnupg apt-transport-https lsb-release v4l-utils
  run sudo mkdir -p /etc/apt/keyrings

  curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
    gpg --dearmor | \
    sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null

  echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null

  run sudo apt update
  run sudo apt install -y \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-udev-rules \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-usb-cam
}

install_system_build_dependencies() {
  local apt_packages=(
    ros-humble-moveit
    ros-humble-gripper-controllers
    ros-humble-position-controllers
    ros-humble-joint-state-broadcaster
    ros-humble-joint-trajectory-controller
    ros-humble-xacro
    ros-humble-hardware-interface
    ros-humble-controller-manager
    ros-humble-ros2controlcli
    ros-humble-moveit-plugins
    ros-humble-moveit-ros-perception
    ros-humble-pinocchio
    ros-humble-nav2-core
    ros-humble-nav2-planner
    ros-humble-nav2-controller
    ros-humble-nav2-bt-navigator
    ros-humble-nav2-lifecycle-manager
    ros-humble-nav2-map-server
    ros-humble-nav2-rviz-plugins
    ros-humble-robot-localization
    ros-humble-ublox-gps
    ros-humble-message-filters
    ros-humble-cv-bridge
    ros-humble-image-transport
    ros-humble-pcl-ros
    qtbase5-dev
    libxcb-cursor0
    libxcb-xinerama0
    libxcb-icccm4
    libxcb-keysyms1
    libxcb-render-util0
    can-utils
    git
    python3-vcstool
    python3-pip
    python3-yaml
    python3-tk
    python3-websockets
  )
  local python_packages=(
    python-can
    pyside6
    openarmx_arm_driver
    casadi
    numpy==1.26.4
    typing_extensions
  )

  echo "Installing OpenFlex system build dependencies:"
  printf '  %s\n' "${apt_packages[@]}"
  request_sudo_for_install

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo apt update -y"
    echo "+ sudo apt-get install -y ${apt_packages[*]}"
    echo "+ python3 -m pip install -i ${PYPI_INDEX_URL} ${python_packages[*]}"
    return
  fi

  run sudo apt update -y
  run sudo apt-get install -y "${apt_packages[@]}"

  echo "Installing OpenFlex Python build dependencies from:"
  echo "  ${PYPI_INDEX_URL}"
  printf '  %s\n' "${python_packages[@]}"
  python3 -m pip install -i "${PYPI_INDEX_URL}" "${python_packages[@]}"
}

install_python_drivers() {
  local whl_files=()
  local python_deps=(
    "numpy==1.26.4"
    "PyYAML==6.0.3"
    "python-can==4.6.1"
  )
  mapfile -t whl_files < <(find "${DRIVER_DIR}" -maxdepth 1 -type f -name "*.whl" | sort)

  if [[ "${#whl_files[@]}" -eq 0 ]]; then
    echo "No Python driver packages (.whl) found, skipping."
    return
  fi

  echo "Installing Python driver packages:"
  printf '  %s\n' "${whl_files[@]}"
  echo "Installing Python driver dependencies from:"
  echo "  ${PYPI_INDEX_URL}"
  printf '  %s\n' "${python_deps[@]}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ pip3 install --user -i ${PYPI_INDEX_URL} --prefer-binary --only-binary=:all: --no-deps ${python_deps[*]}"
    echo "+ pip3 install --user --force-reinstall --no-deps ${whl_files[*]}"
    return
  fi

  # Install dependencies first with pinned compatibility, then install local
  # OpenFlex wheels without letting pip resolve the pinocchio/NumPy dependency
  # chain. That chain is shared with ROS/teleop packages and must not be
  # rewritten by this installer.
  pip3 install --user -i "${PYPI_INDEX_URL}" --prefer-binary --only-binary=:all: --no-deps "${python_deps[@]}"
  pip3 install --user --force-reinstall --no-deps "${whl_files[@]}"

  if [[ $? -eq 0 ]]; then
    echo "✓ Python drivers installed successfully"
  else
    echo "ERROR: Failed to install Python drivers" >&2
    exit 1
  fi
}

install_vr_software() {
  local apk_dir="${OPENFLEX_ROOT}/openflex_vr_apk/apk"
  local pico_apk="${apk_dir}/pico/OpenFlex.apk"
  local quest_apk="${apk_dir}/quest/openarmx-vr-quest.apk"
  local apk_path=""
  local answer
  local device_choice

  require_not_root
  echo "请选择要安装的 VR 软件:"
  echo "  1. Pico"
  echo "  2. Quest"
  read -r -p "请输入选项 [1/2]: " device_choice
  case "${device_choice}" in
    1|pico|Pico|PICO)
      apk_path="${pico_apk}"
      ;;
    2|quest|Quest|QUEST)
      apk_path="${quest_apk}"
      ;;
    *)
      echo "ERROR: invalid VR device option: ${device_choice}" >&2
      exit 2
      ;;
  esac
  require_file "${apk_path}"

  echo "安装 VR 软件前，请先确认已使用 USB 3.0 线连接主机与 VR 设备。"
  read -r -p "是否已经连接？[yes/no]: " answer
  case "${answer,,}" in
    yes|y)
      ;;
    *)
      echo "已取消 VR 软件安装。"
      exit 0
      ;;
  esac

  if command -v adb >/dev/null 2>&1; then
    echo "adb is already installed: $(command -v adb)"
  else
    request_sudo_for_install
    run sudo apt install -y adb
  fi
  run adb install "${apk_path}"
}

detect_kernel_gcc_binary() {
  local gcc_version=""
  local config_file="/boot/config-$(uname -r)"

  if [[ -f "${config_file}" ]]; then
    gcc_version="$(grep -E '^CONFIG_GCC_VERSION=' "${config_file}" | head -n1 | cut -d= -f2 || true)"
  fi

  case "${gcc_version}" in
    120300|12*)
      printf '%s\n' "gcc-12"
      ;;
    110*|11*)
      printf '%s\n' "gcc-11"
      ;;
    100*|10*)
      printf '%s\n' "gcc-10"
      ;;
    *)
      printf '%s\n' "gcc"
      ;;
  esac
}

install_kcan_dependencies() {
  local kernel_release
  local gcc_bin
  local apt_packages

  kernel_release="$(uname -r)"
  gcc_bin="$(detect_kernel_gcc_binary)"
  apt_packages=(
    build-essential
    g++
    dkms
    wget
    unzip
    libpopt-dev
    "linux-headers-${kernel_release}"
  )

  if [[ "${gcc_bin}" != "gcc" ]]; then
    apt_packages+=("${gcc_bin}")
  fi

  echo "Installing KCAN dependencies:"
  printf '  %s\n' "${apt_packages[@]}"
  request_sudo_for_install

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo apt update"
    echo "+ sudo apt install -y ${apt_packages[*]}"
    return
  fi

  run sudo apt update
  run sudo apt install -y "${apt_packages[@]}"
}

maybe_uninstall_pcan_driver() {
  local answer
  local pcan_dir=""

  echo "如曾安装 PCAN 驱动，建议先卸载，避免和 KCAN 冲突。"
  read -r -p "是否需要卸载 PCAN 驱动？[yes/no]: " answer
  case "${answer,,}" in
    yes|y)
      ;;
    *)
      echo "跳过 PCAN 驱动卸载。"
      return
      ;;
  esac

  echo "Checking PCAN driver after user confirmation..."
  if lsmod | grep -q '^pcan\b'; then
    run sudo modprobe -r pcan
  else
    echo "pcan kernel module is not loaded."
  fi

  pcan_dir="$(find "${HOME}" -maxdepth 2 -type d -name 'peak-linux-driver-*' | sort | tail -n1 || true)"
  if [[ -z "${pcan_dir}" ]]; then
    echo "No peak-linux-driver-* directory found under ${HOME}; skipping PCAN make uninstall."
    return
  fi

  echo "Uninstalling PCAN driver from:"
  echo "  ${pcan_dir}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo make -C ${pcan_dir} uninstall"
    return
  fi
  run sudo make -C "${pcan_dir}" uninstall
}

prepare_kcan_sdk() {
  echo "Preparing KCAN SDK:"
  echo "  zip: ${KCAN_SDK_ZIP}"
  echo "  dir: ${KCAN_SDK_DIR}"

  if [[ -d "${KCAN_SDK_DIR}" ]]; then
    echo "Using existing KCAN SDK directory."
    return
  fi

  if [[ ! -f "${KCAN_SDK_ZIP}" ]]; then
    echo "Downloading KCAN SDK:"
    echo "  ${KCAN_SDK_URL}"
    if [[ "${DRY_RUN}" -eq 1 ]]; then
      echo "+ wget -O ${KCAN_SDK_ZIP} ${KCAN_SDK_URL}"
    else
      run wget -O "${KCAN_SDK_ZIP}" "${KCAN_SDK_URL}"
    fi
  else
    echo "Using existing KCAN SDK zip."
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ unzip -o ${KCAN_SDK_ZIP} -d ${HOME}"
    return
  fi

  run unzip -o "${KCAN_SDK_ZIP}" -d "${HOME}"
  require_dir "${KCAN_SDK_DIR}"
}

build_kcan_sdk_once() {
  echo "Building KCAN SDK once:"
  echo "  ${KCAN_SDK_DIR}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo sed -i 's/-ftrivial-auto-var-init=zero//g' /usr/src/linux-headers-$(uname -r)/Makefile"
    echo "+ sudo chmod +x ${KCAN_SDK_DIR}/build.sh"
    echo "+ (cd ${KCAN_SDK_DIR} && ./build.sh)"
    return
  fi

  require_dir "${KCAN_SDK_DIR}"
  require_file "${KCAN_SDK_DIR}/build.sh"

  run sudo sed -i 's/-ftrivial-auto-var-init=zero//g' "/usr/src/linux-headers-$(uname -r)/Makefile"
  run sudo chmod +x "${KCAN_SDK_DIR}/build.sh"
  (cd "${KCAN_SDK_DIR}" && ./build.sh)
}

write_kcan_initial_dkms_conf() {
  echo "Creating the initial KCAN DKMS config from the reference procedure:"
  echo "  ${KCAN_DKMS_SRC}/dkms.conf"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo mkdir -p ${KCAN_DKMS_SRC}"
    echo "+ sudo tee ${KCAN_DKMS_SRC}/dkms.conf"
    return
  fi

  run sudo mkdir -p "${KCAN_DKMS_SRC}"
  cat << 'EOF' | sudo tee "${KCAN_DKMS_SRC}/dkms.conf" >/dev/null
PACKAGE_NAME="kunhong-linux-driver"
PACKAGE_VERSION="8.20.0"
CLEAN="make clean"
MAKE[0]="sed -i 's/-ftrivial-auto-var-init=zero//g' /usr/src/linux-headers-${kernelver}/Makefile 2>/dev/null || true; make DKMS_KERNEL_DIR=$kernel_source_dir MOD=MODVERSIONS PAR=NO_PARPORT_SUBSYSTEM USB=USB_SUPPORT PCI=PCI_SUPPORT PCIEC=PCIEC_SUPPORT ISA=ISA_SUPPORT DNG=NO_DONGLE_SUPPORT PCC=NO_PCCARD_SUPPORT NET=NETDEV_SUPPORT RT=NO_RT"
BUILT_MODULE_NAME[0]="kcan"
BUILT_MODULE_LOCATION[0]="."
DEST_MODULE_LOCATION[0]="/updates"
AUTOINSTALL="yes"
EOF
}

write_kcan_final_dkms_conf() {
  echo "Writing the final KCAN DKMS config from the reference procedure:"
  echo "  ${KCAN_DKMS_SRC}/dkms.conf"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo tee ${KCAN_DKMS_SRC}/dkms.conf"
    return
  fi

  cat << 'EOF' | sudo tee "${KCAN_DKMS_SRC}/dkms.conf" >/dev/null
PACKAGE_NAME="kunhong-linux-driver"
PACKAGE_VERSION="8.20.0"
CLEAN="make clean"
# 优化：仅修改当前编译目录的临时Makefile（不改动系统文件），同时指定正确的源码路径
MAKE[0]="cd ${dkms_tree}/${PACKAGE_NAME}/${PACKAGE_VERSION}/source; sed -i 's/-ftrivial-auto-var-init=zero//g' /usr/src/linux-headers-${kernelver}/Makefile 2>/dev/null || true; make DKMS_KERNEL_DIR=${kernel_source_dir} MOD=MODVERSIONS PAR=NO_PARPORT_SUBSYSTEM USB=USB_SUPPORT PCI=PCI_SUPPORT PCIEC=PCIEC_SUPPORT ISA=ISA_SUPPORT DNG=NO_DONGLE_SUPPORT PCC=NO_PCCARD_SUPPORT NET=NETDEV_SUPPORT RT=NO_RT"
BUILT_MODULE_NAME[0]="kcan"
BUILT_MODULE_LOCATION[0]="."
DEST_MODULE_LOCATION[0]="/updates"
AUTOINSTALL="yes"
EOF
}

install_kcan_dkms() {
  local driver_src="${KCAN_SDK_DIR}/driver"

  echo "Installing KCAN DKMS source:"
  echo "  ${KCAN_DKMS_SRC}"
  request_sudo_for_install

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ sudo mkdir -p ${KCAN_DKMS_SRC}"
    echo "+ sudo cp -r ${driver_src}/* ${KCAN_DKMS_SRC}/"
    write_kcan_final_dkms_conf
    echo "+ sudo dkms add -m ${KCAN_DKMS_NAME} -v ${KCAN_DKMS_VERSION}"
    echo "+ sudo dkms build -m ${KCAN_DKMS_NAME} -v ${KCAN_DKMS_VERSION}"
    echo "+ sudo dkms install -m ${KCAN_DKMS_NAME} -v ${KCAN_DKMS_VERSION}"
    echo "+ dkms status"
    return
  fi

  require_dir "${driver_src}"

  run sudo mkdir -p "${KCAN_DKMS_SRC}"
  run sudo cp -r "${driver_src}/"* "${KCAN_DKMS_SRC}/"
  write_kcan_final_dkms_conf

  run sudo dkms add -m "${KCAN_DKMS_NAME}" -v "${KCAN_DKMS_VERSION}"
  run sudo dkms build -m "${KCAN_DKMS_NAME}" -v "${KCAN_DKMS_VERSION}"
  run sudo dkms install -m "${KCAN_DKMS_NAME}" -v "${KCAN_DKMS_VERSION}"
  run dkms status
}

verify_kcan_loaded() {
  echo "Loading KCAN kernel module..."
  run sudo modprobe kcan

  echo "Verifying KCAN kernel module:"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ lsmod | grep kcan"
    echo "+ ip link show"
    return
  fi

  if lsmod | grep -q '^kcan\b'; then
    lsmod | grep '^kcan\b'
  else
    echo "ERROR: KCAN verification failed: lsmod does not contain kcan." >&2
    exit 1
  fi

  echo
  echo "CAN interface information (reference only; final KCAN check is lsmod):"
  ip link show
}

enable_kcan_autoload() {
  echo "Enabling KCAN autoload:"
  echo "  /etc/modules-load.d/kcan.conf"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ echo kcan | sudo tee /etc/modules-load.d/kcan.conf"
    return
  fi

  echo "kcan" | sudo tee /etc/modules-load.d/kcan.conf >/dev/null
}

install_kcan_driver() {
  require_not_root

  install_kcan_dependencies
  prepare_kcan_sdk
  build_kcan_sdk_once
  write_kcan_initial_dkms_conf
  install_kcan_dkms
  verify_kcan_loaded
  enable_kcan_autoload

  echo
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "KCAN driver dry-run completed."
  else
    echo "KCAN driver installed successfully."
  fi
  echo "Final verification standard: lsmod contains kcan."
}

check_installed_driver_paths() {
  echo "Checking installed driver dependency paths..."

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ check /opt/openflex/acados/include and /opt/openflex/acados/lib/libacados.so"
    echo "+ check /usr/local/include/livox_lidar_api.h and /usr/local/lib/liblivox_lidar_sdk_shared.so"
    echo "+ check openarmx_can CMake config under /usr/share or /usr/lib"
    echo "+ check openflex_can CMake config under /usr/share or /usr/lib"
    return
  fi

  if [[ ! -d /opt/openflex/acados/include || ! -f /opt/openflex/acados/lib/libacados.so ]]; then
    echo "ERROR: acados not found. Expected /opt/openflex/acados from openflex-acados_*.deb." >&2
    exit 1
  fi

  require_file /usr/local/include/livox_lidar_api.h
  require_file /usr/local/lib/liblivox_lidar_sdk_shared.so

  local openarmx_can_ok=0
  for config_path in \
    /usr/share/openarmx_can/cmake/openarmx_canConfig.cmake \
    /usr/lib/cmake/openarmx_can/openarmx_canConfig.cmake \
    /usr/lib/x86_64-linux-gnu/cmake/OpenArmXCAN/OpenArmXCANConfig.cmake; do
    if [[ -f "${config_path}" ]]; then
      openarmx_can_ok=1
      break
    fi
  done
  if [[ "${openarmx_can_ok}" -ne 1 ]]; then
    echo "ERROR: openarmx_can CMake config was not found under /usr." >&2
    exit 1
  fi

  local openflex_can_ok=0
  for config_path in \
    /usr/share/openflex_can/cmake/openflex_canConfig.cmake \
    /usr/lib/cmake/openflex_can/openflex_canConfig.cmake \
    /usr/lib/x86_64-linux-gnu/cmake/openflex_can/openflex_canConfig.cmake; do
    if [[ -f "${config_path}" ]]; then
      openflex_can_ok=1
      break
    fi
  done
  if [[ "${openflex_can_ok}" -ne 1 ]]; then
    echo "ERROR: openflex_can CMake config was not found under /usr." >&2
    exit 1
  fi
}

desktop_dir() {
  if [[ -n "${XDG_DESKTOP_DIR:-}" ]]; then
    printf '%s\n' "${XDG_DESKTOP_DIR}"
  elif [[ -d "${HOME}/桌面" ]]; then
    printf '%s\n' "${HOME}/桌面"
  else
    printf '%s\n' "${HOME}/Desktop"
  fi
}

create_desktop_launcher() {
  local icon_path="${WORKSPACE_DIR}/src/openflex_integrated/openflex_gui/openflex_gui/openflex_vr.svg"
  local desktop_path

  require_file "${icon_path}"
  desktop_path="$(desktop_dir)/OpenFlex控制台.desktop"

  echo "Creating desktop launcher:"
  echo "  ${desktop_path}"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    echo "+ mkdir -p $(dirname "${desktop_path}")"
    echo "+ write ${desktop_path}"
    echo "+ chmod +x ${desktop_path}"
    return
  fi

  mkdir -p "$(dirname "${desktop_path}")"
  {
    printf '%s\n' '[Desktop Entry]'
    printf '%s\n' 'Version=1.0'
    printf '%s\n' 'Type=Application'
    printf '%s\n' 'Name=OpenFlex 控制台'
    printf 'Exec=bash -c "source %q && source %q && ros2 run openflex_gui openflex_gui"\n' \
      "${ROS_SETUP}" \
      "${WORKSPACE_DIR}/install/setup.bash"
    printf 'Icon=%s\n' "${icon_path}"
    printf '%s\n' 'Terminal=false'
    printf '%s\n' 'Categories=Robotics;'
  } > "${desktop_path}"
  chmod +x "${desktop_path}"
}

package_exists() {
  local package_name="$1"
  colcon list --base-paths src --names-only | grep -Fxq "${package_name}"
}

build_packages() {
  local label="$1"
  shift
  local packages=()
  local package_name

  for package_name in "$@"; do
    if package_exists "${package_name}"; then
      packages+=("${package_name}")
    else
      echo "WARN: package not found, skipping: ${package_name}" >&2
    fi
  done

  if [[ "${#packages[@]}" -eq 0 ]]; then
    echo "Skipping empty build step: ${label}"
    return
  fi

  echo
  echo "==> ${label}: ${packages[*]}"
  run colcon build \
    --packages-select "${packages[@]}" \
    --parallel-workers "${PARALLEL_WORKERS}" \
    --cmake-args -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

  if [[ "${DRY_RUN}" -eq 0 ]]; then
    source_if_exists "${WORKSPACE_DIR}/install/setup.bash"
  fi
}

build_one_by_one() {
  local label="$1"
  shift
  local package_name

  for package_name in "$@"; do
    build_packages "${label}: ${package_name}" "${package_name}"
  done
}

build_optional_one_by_one() {
  local label="$1"
  shift
  local package_name
  local built_any=0

  for package_name in "$@"; do
    if package_exists "${package_name}"; then
      built_any=1
      build_packages "${label}: ${package_name}" "${package_name}"
    fi
  done

  if [[ "${built_any}" -eq 0 ]]; then
    echo "Skipping optional build step: ${label}"
  fi
}

build_openflex_workspace() {
  # These packages are independent workspace foundations and can be built
  # separately. They are intentionally built first, one by one.
  build_one_by_one "single base package" \
    lift_slide_msgs \
    lift_slide_description \
    interface \
    swerve_description \
    openarmx_description \
    openarmx_head_description \
    openflex_vr_bridge \
    pcd2pgm \
    pointcloud_to_laserscan

  # Driver packages need the debs installed before this stage.
  build_one_by_one "single external-driver package" \
    openarmx_hardware \
    openarmx_head_hardware \
    livox_ros_driver2 \
    nmpc_controller

  # Runtime groups are compiled together because they are normally changed and
  # deployed as one subsystem. Colcon still resolves their internal order.
  build_packages "lift runtime group" \
    lift_slide_driver \
    lift_slide_panel \
    lift_slide_bringup

  build_packages "swerve runtime group" \
    swerve_hardware \
    swerve_controller \
    swerve_bringup

  build_packages "swerve tool panels group" \
    swerve_base_panel

  build_packages "openarmx runtime group" \
    openarmx_bringup \
    openarmx \
    openarmx_gravity_comp \
    openarmx_bimanual_moveit_config

  build_packages "openarmx dexterous hand runtime group" \
    hands_description \
    hands_hardware \
    hands_bringup \
    openarmx_hand_hardware \
    openarmx_hand_description \
    openarmx_hand_bringup \
    openarmx_hands_hig \
    openarmx_hands_bridge \
    openarmx_hand_gui

  build_packages "openarmx tool panels group" \
    openarmx_joint_slider_panel \
    openarmx_gripper_panel \
    openarmx_kp_kd_panel \
    openarmx_battery_monitor \
    openarmx_preview_bringup \
    openarmx_ik_control_panel

  build_packages "head runtime group" \
    openarmx_head_bringup \
    openarmx_head_joint_slider_panel \
    openarmx_head_vision_h264

  build_one_by_one "single python/tool package" \
    openarmx_teach \
    openarmx_teleop_vr \
    openarmx_head_teleop_vr_pico \
    openflex_teleop_exo \
    openflex_teleop_exo_speed_panel \
    openflex_gui \
    vikit_py

  # Mapping/localization packages depend on livox_ros_driver2 and sometimes on
  # each other. Build the shared vikit pair together, then the consumers.
  build_packages "vikit mapping support group" \
    vikit_common \
    vikit_ros

  build_one_by_one "single mapping package" \
    fastlio2 \
    icp_registration \
    hba \
    pgo \
    fast_livo

  build_optional_one_by_one "single optional VLA package" \
    lerobot_robot_openflex_follower_ros2 \
    lerobot_teleoperator_openflex_leader_ros2

  build_packages "integrated final group" \
    openarmx_integrated_description \
    swerve_navigation \
    openarmx_integrated_bringup
}

install_openflex_environment() {
  if [[ "${SKIP_INSTALL}" -eq 0 ]]; then
    require_dir "${DRIVER_DIR}"
    install_camera_dependencies
    install_system_build_dependencies
    install_driver_debs
    install_python_drivers
  else
    echo "Skipping system/deb/Python install."
  fi
}

install_openflex() {
  require_not_root
  prepare_openflex_sources
  ensure_workspace
  cd "${WORKSPACE_DIR}"

  ensure_user_camera_config
  ensure_user_lidar_config

  install_openflex_environment

  if [[ "${DRY_RUN}" -eq 0 ]]; then
    source_required "${ROS_SETUP}"
    source_if_exists "${WORKSPACE_DIR}/install/setup.bash"
  else
    echo "+ source ${ROS_SETUP}"
    echo "+ source ${WORKSPACE_DIR}/install/setup.bash if it exists"
  fi

  check_installed_driver_paths

  if [[ "${SKIP_BUILD}" -eq 1 ]]; then
    echo "Skipping colcon build."
    if [[ "${SKIP_DESKTOP}" -eq 0 ]]; then
      create_desktop_launcher
    else
      echo "Skipping desktop launcher."
    fi
    exit 0
  fi

  build_openflex_workspace

  if [[ "${SKIP_DESKTOP}" -eq 0 ]]; then
    create_desktop_launcher
  else
    echo "Skipping desktop launcher."
  fi

  echo
  echo "Done. Workspace setup:"
  echo "  source ${WORKSPACE_DIR}/install/setup.bash"
}

install_openflex_environment_only() {
  require_not_root
  ensure_workspace
  cd "${WORKSPACE_DIR}"

  install_openflex_environment

  echo
  echo "OpenFlex environment installation completed."
}

compile_openflex() {
  require_not_root
  ensure_workspace
  cd "${WORKSPACE_DIR}"

  if [[ "${DRY_RUN}" -eq 0 ]]; then
    source_required "${ROS_SETUP}"
    source_if_exists "${WORKSPACE_DIR}/install/setup.bash"
  else
    echo "+ source ${ROS_SETUP}"
    echo "+ source ${WORKSPACE_DIR}/install/setup.bash if it exists"
  fi

  check_installed_driver_paths
  build_openflex_workspace

  echo
  echo "Done. Workspace setup:"
  echo "  source ${WORKSPACE_DIR}/install/setup.bash"
}

install_desktop_gui() {
  require_not_root
  require_dir "${WORKSPACE_DIR}/src"
  create_desktop_launcher
}

sync_sources_only() {
  prepare_openflex_sources
  echo
  echo "Source repositories are ready under:"
  echo "  ${WORKSPACE_DIR}/src"
}

main() {
  choose_mode
  case "${MODE}" in
    openflex)
      install_openflex
      ;;
    desktop)
      install_desktop_gui
      ;;
    vr)
      install_vr_software
      ;;
    camera)
      configure_cameras
      ;;
    lidar)
      configure_lidar
      ;;
    environment)
      install_openflex_environment_only
      ;;
    compile)
      compile_openflex
      ;;
    kcan)
      install_kcan_driver
      ;;
    sync-source)
      sync_sources_only
      ;;
    *)
      echo "ERROR: invalid mode: ${MODE}" >&2
      exit 2
      ;;
  esac
}

main "$@"
