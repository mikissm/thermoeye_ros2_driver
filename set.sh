#!/usr/bin/env bash
# Extract the vendor SDK privately; never run vendor maintainer scripts.
set -euo pipefail
SDK_VERSION=2.1.0
REPO_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
SDK_DIR="${HOME}/.local/opt/tmsdk/${SDK_VERSION}"
SDK_DEB=''
INSTALL_DEPS=0
RUN_ROSDEP=0
YES=0
usage() {
  cat <<'HELP'
Usage: ./set.sh [options]
  --sdk-dir PATH    SDK extraction directory (default ~/.local/opt/tmsdk/2.1.0)
  --sdk-deb FILE    Use a previously downloaded official Ubuntu 24.04 SDK .deb
  --install-deps    Install build, GUI and SDK runtime dependencies using apt
  --rosdep          Install package dependencies using configured rosdep
  --yes             Accept default path and apt confirmation (for automation)
  -h, --help        Show this help

Supported: Ubuntu 24.04, ROS 2 Jazzy, amd64/arm64. ROS itself must already
be installed. No host packages or shell startup files change without options.
Source .tmsdk/env.sh before building at your colcon workspace root.
HELP
}
fail() { printf 'Error: %s\n' "$*" >&2; exit 1; }
while (($#)); do
  case "$1" in
    --sdk-dir|--sdk-deb)
      (($# >= 2)) || fail "$1 requires a value"
      [[ -n "$2" && "$2" != --* ]] || fail "$1 requires a value"
      if [[ "$1" == --sdk-dir ]]; then SDK_DIR=$2; else SDK_DEB=$2; fi
      shift 2 ;;
    --install-deps) INSTALL_DEPS=1; shift ;;
    --rosdep) RUN_ROSDEP=1; shift ;;
    --yes) YES=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) fail "Unknown option: $1" ;;
  esac
done
command -v dpkg >/dev/null || fail 'dpkg is required'
ARCH=$(dpkg --print-architecture)
case "$ARCH" in
  amd64) SDK_SHA256=d34baf3013785ee4b33b882bd26d876bd45f549842175119a3be5609a007a57b ;;
  arm64) SDK_SHA256=8463baa1945c4eff83439c513ff777643264f8dc53c22144433c4ac023708368 ;;
  *) fail "Unsupported architecture: $ARCH" ;;
esac
# shellcheck disable=SC1091
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || fail 'Ubuntu 24.04 is required for this SDK/Jazzy installer'
[[ -z "${ROS_DISTRO:-}" || "$ROS_DISTRO" == jazzy ]] || fail 'Source ROS 2 Jazzy, not another ROS distribution'
[[ -f /opt/ros/jazzy/setup.bash ]] || fail 'Install ROS 2 Jazzy first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html'
if [[ -t 0 && $YES == 0 ]]; then
  read -r -p "SDK directory [$SDK_DIR]: " CHOICE
  SDK_DIR=${CHOICE:-$SDK_DIR}
fi
[[ "$SDK_DIR" != *$'\n'* && "$SDK_DIR" != *:* ]] || fail 'SDK directory must not contain colons or newlines'
SDK_DIR=$(realpath -m -- "$SDK_DIR")
[[ "$SDK_DIR" != / && "$SDK_DIR" != /usr && "$SDK_DIR" != "$HOME" ]] || fail 'Choose a dedicated SDK directory'
SUDO=()
if ((INSTALL_DEPS || RUN_ROSDEP)); then
  if ((EUID != 0)); then command -v sudo >/dev/null || fail 'sudo is required'; SUDO=(sudo); fi
fi
if ((INSTALL_DEPS)); then
  APT_ARGS=(); if ((YES)); then APT_ARGS=(-y); fi
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" apt-get install "${APT_ARGS[@]}" curl ca-certificates build-essential cmake \
    python3-colcon-common-extensions python3-rosdep python3-pyqt5 libopencv-dev \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-rclcpp ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs ros-jazzy-std-msgs ros-jazzy-std-srvs \
    ros-jazzy-ament-cmake ros-jazzy-ament-index-python ros-jazzy-launch-ros \
    libudev1 libdc1394-25 libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 \
    libavcodec60 libavformat60 libswscale7 libopenexr-3-1-30
fi
if ((RUN_ROSDEP)); then
  command -v rosdep >/dev/null || fail 'Install/configure rosdep first (or use --install-deps)'
  ROSDEP_ARGS=(); if ((YES)); then ROSDEP_ARGS=(-y); fi
  rosdep install --from-paths "$REPO_DIR" --ignore-src --rosdistro jazzy "${ROSDEP_ARGS[@]}"
fi
valid_sdk() {
  [[ -f "$1/usr/include/TmSDK/TmLocalCamera.hxx" &&
     -f "$1/usr/lib/TmSDK/libTmCore.${SDK_VERSION}.so" &&
     -f "$1/usr/lib/TmSDK/libopencv_world.so" &&
     -f "$1/usr/lib/TmSDK/libTmRtspClient.1.1.4.so" ]]
}
TEMP_DIR=''
cleanup() { [[ -z "$TEMP_DIR" ]] || rm -rf -- "$TEMP_DIR"; }
trap cleanup EXIT
if valid_sdk "$SDK_DIR"; then
  printf 'Using existing SDK: %s\n' "$SDK_DIR"
else
  if [[ -e "$SDK_DIR" && -n "$(ls -A -- "$SDK_DIR")" ]]; then
    fail "SDK directory is nonempty but incomplete; choose another directory: $SDK_DIR"
  fi
  mkdir -p -- "$(dirname -- "$SDK_DIR")"
  TEMP_DIR=$(mktemp -d "$(dirname -- "$SDK_DIR")/.tmsdk-install.XXXXXX")
  if [[ -z "$SDK_DEB" ]]; then
    command -v curl >/dev/null || fail 'curl is required (use --install-deps or --sdk-deb)'
    SDK_DEB="$TEMP_DIR/sdk.deb"
    ASSET="tmsdk-${SDK_VERSION}-lib-Ubuntu-24.04-${ARCH}.deb"
    URL="https://github.com/ThermoEye/TmSDK/releases/download/${SDK_VERSION}/${ASSET}"
    printf 'Downloading official SDK: %s\n' "$URL"
    curl --fail --location --proto '=https' --tlsv1.2 --retry 3 --connect-timeout 20 \
      --output "$SDK_DEB" "$URL"
  fi
  [[ -f "$SDK_DEB" ]] || fail "SDK package not found: $SDK_DEB"
  ACTUAL_SHA256=$(sha256sum -- "$SDK_DEB")
  [[ "${ACTUAL_SHA256%% *}" == "$SDK_SHA256" ]] || fail 'SDK SHA256 differs from the official 2.1.0 Ubuntu 24.04 asset'
  [[ "$(dpkg-deb -f "$SDK_DEB" Package)" == tmsdk ]] || fail 'Expected the official tmsdk package'
  [[ "$(dpkg-deb -f "$SDK_DEB" Version)" == "$SDK_VERSION" ]] || fail "Expected SDK version $SDK_VERSION"
  [[ "$(dpkg-deb -f "$SDK_DEB" Architecture)" == "$ARCH" ]] || fail "SDK architecture must be $ARCH"
  dpkg-deb -x "$SDK_DEB" "$TEMP_DIR/extracted"
  valid_sdk "$TEMP_DIR/extracted" || fail 'SDK package does not contain the expected headers/libraries'
  # Only replace an absent or empty target; never overwrite an existing installation.
  if [[ -d "$SDK_DIR" ]]; then rmdir -- "$SDK_DIR"; fi
  mv -T -- "$TEMP_DIR/extracted" "$SDK_DIR"
fi
mkdir -p -- "$REPO_DIR/.tmsdk"
ENV_TMP=$(mktemp "$REPO_DIR/.tmsdk/env.sh.XXXXXX")
{
  printf '# Generated by set.sh; local configuration, do not commit.\n'
  printf 'export TMSDK_ROOT=%q\n' "$SDK_DIR"
  cat <<'ENV'
case ":${LD_LIBRARY_PATH:-}:" in
  *":$TMSDK_ROOT/usr/lib/TmSDK:"*) ;;
  *) export LD_LIBRARY_PATH="$TMSDK_ROOT/usr/lib/TmSDK${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac
ENV
} > "$ENV_TMP"
mv -- "$ENV_TMP" "$REPO_DIR/.tmsdk/env.sh"
printf '\nSDK ready. In each new terminal:\n  source /opt/ros/jazzy/setup.bash\n  source %q\nThen build from your colcon workspace root:\n  colcon build --packages-select thermal_cam_driver --cmake-clean-cache\n  source install/setup.bash\n' "$REPO_DIR/.tmsdk/env.sh"
printf 'No camera was opened. USB device permissions may require local configuration.\n'
