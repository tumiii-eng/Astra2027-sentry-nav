#!/usr/bin/env bash
# 拉取上游依赖仓库到 src/，并打上本工程需要的补丁。
# 幂等：已存在的仓库会跳过克隆，补丁重复执行不会重复应用。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="$REPO_ROOT/src"

if ! command -v vcs >/dev/null 2>&1; then
  echo "缺少 vcstool，先装：sudo apt install python3-vcstool" >&2
  exit 1
fi

echo ">>> 拉取上游仓库到 $SRC_DIR"
vcs import "$SRC_DIR" < "$REPO_ROOT/upstream.repos"

echo ">>> 拉取 pb2025_sentry_nav 的子模块"
git -C "$SRC_DIR/pb2025_sentry_nav" submodule update --init --recursive

echo ">>> 打补丁：移除 ament_auto_package 的 USE_SCOPED_HEADER_INSTALL_DIR"
# 上游用的 ament_cmake_auto 版本不支持这个选项，留着会编译失败。
PATCH_TARGETS=(
  # pb2025_sentry_nav 主仓库内的包
  "pb2025_sentry_nav/fake_vel_transform/CMakeLists.txt"
  "pb2025_sentry_nav/ign_sim_pointcloud_tool/CMakeLists.txt"
  "pb2025_sentry_nav/loam_interface/CMakeLists.txt"
  "pb2025_sentry_nav/sensor_scan_generation/CMakeLists.txt"
  # pb2025_sentry_nav 的子模块
  "pb2025_sentry_nav/pb_nav2_plugins/CMakeLists.txt"
  "pb2025_sentry_nav/pb_omni_pid_pursuit_controller/CMakeLists.txt"
  "pb2025_sentry_nav/pb_teleop_twist_joy/CMakeLists.txt"
  "pb2025_sentry_nav/small_gicp_relocalization/CMakeLists.txt"
  # 仿真器
  "rmu_gazebo_simulator/rmu_gazebo_simulator/CMakeLists.txt"
)
for rel in "${PATCH_TARGETS[@]}"; do
  f="$SRC_DIR/$rel"
  if [[ ! -f "$f" ]]; then
    echo "  跳过（文件不存在）: $rel"
    continue
  fi
  if grep -q "USE_SCOPED_HEADER_INSTALL_DIR" "$f"; then
    sed -i '/USE_SCOPED_HEADER_INSTALL_DIR/d' "$f"
    echo "  已打补丁: $rel"
  else
    echo "  已是最新: $rel"
  fi
done

echo
echo ">>> 完成。接下来："
echo "    docker compose up -d"
echo "    docker compose exec dev bash -lc 'cd /workspace && colcon build --symlink-install'"
