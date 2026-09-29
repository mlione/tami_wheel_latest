#!/usr/bin/env bash
# Read-only ROS diagnostics and rosbag recording. Never starts navigation or BLE.
set -euo pipefail

usage() {
  printf '%s\n' \
    '用法：bash record_dwa_bag.sh [测试名称] [--light]' \
    '      bash record_dwa_bag.sh --help' \
    '' \
    '不填名称时交互输入；名称可包含英文字母、数字、下划线、点和连字符。' \
    '默认录制控制、反馈、规划点云、候选路径和日志；--light 不录点云和 Marker。' \
    '自动保存运行参数和版本信息；看到 rosbag 开始录制后再进行测试。' \
    '按 Ctrl+C 正常结束录包。仅停止录制，不会停止轮椅或导航节点。'
}

test_name=''
light=false
for argument in "$@"; do
  case "$argument" in
    -h|--help) usage; exit 0 ;;
    --light) light=true ;;
    -*) printf '未知选项：%s\n' "$argument" >&2; usage >&2; exit 2 ;;
    *)
      if [[ -n "$test_name" ]]; then
        printf '只能指定一个测试名称。\n' >&2
        exit 2
      fi
      test_name="$argument"
      ;;
  esac
done

if [[ -z "$test_name" ]]; then
  if [[ -t 0 ]]; then
    read -r -p '本次测试名称（回车默认 dwa_test）：' test_name
  fi
  test_name="${test_name:-dwa_test}"
fi
if [[ ! "$test_name" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]]; then
  printf '测试名称须以字母或数字开头，且不能含空格、斜杠或中文。\n' >&2
  exit 2
fi

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"

# ROS/colcon setup files may reference unset variables; do not use nounset here.
set +u
if [[ -r /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -r "$project_root/install/local_setup.bash" ]]; then
  source "$project_root/install/local_setup.bash"
elif [[ -r "$project_root/install/setup.bash" ]]; then
  source "$project_root/install/setup.bash"
else
  printf '提示：当前项目没有 install 环境，将使用终端已有的 ROS 环境。\n' >&2
fi
set -u

if ! command -v ros2 >/dev/null || ! command -v timeout >/dev/null; then
  printf '找不到 ros2 或 timeout；请安装 ROS 2 Humble 和 coreutils。\n' >&2
  exit 1
fi
if ! timeout 8s ros2 pkg prefix wheel_msgs >/dev/null 2>&1; then
  printf '当前环境找不到 wheel_msgs，无法可靠录制 /perception/output。\n' >&2
  printf '请先构建当前项目并 source install/setup.bash，再运行本脚本。\n' >&2
  exit 1
fi

topics=(
  /cmd_vel
  /odom
  /perception/output
  /dwa/planner_cmd
  /dwa/indoor_test_cmd
  /dwa/local_trajectory
  /dwa/best_path
  /zed/diagnostics/sdk_twist
  /rosout
  /tf
  /tf_static
  /parameter_events
)
if [[ "$light" == false ]]; then
  topics+=(
    /dwa/best_path_marker
    /dwa/candidate_paths
    /dwa/obstacle_cloud
    /perception/debug/right_road_edge
  )
fi

mkdir -p -- "$project_root/bags"
session_dir="$(mktemp -d "$project_root/bags/${test_name}_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
bag_dir="$session_dir/bag"
mkdir -- "$session_dir/parameters"

{
  printf 'test_name: %s\n' "$test_name"
  printf 'prepared_at: %s\n' "$(date --iso-8601=seconds)"
  printf 'project_root: %s\n' "$project_root"
  printf 'light_mode: %s\n' "$light"
  printf 'bag_directory: %s\n' "$bag_dir"
  printf 'ROS_DOMAIN_ID: %s\n' "${ROS_DOMAIN_ID:-0}"
  printf 'RMW_IMPLEMENTATION: %s\n' "${RMW_IMPLEMENTATION:-default}"
  printf '\nTopics:\n'
  printf '%s\n' "${topics[@]}"
  if command -v git >/dev/null && git -C "$project_root" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    printf '\nGit HEAD:\n'
    git -C "$project_root" rev-parse HEAD
    printf '\nGit status (not a copy of uncommitted changes):\n'
    git -C "$project_root" status --short
  fi
} > "$session_dir/session_info.txt"

source_config="$project_root/src/wheel_perception/config/params.yaml"
installed_config="$project_root/install/wheel_perception/share/wheel_perception/config/params.yaml"
if [[ -r "$source_config" ]]; then
  cp -- "$source_config" "$session_dir/parameters/source_params.yaml"
fi
if [[ -r "$installed_config" ]]; then
  cp -- "$installed_config" "$session_dir/parameters/installed_params.yaml"
fi

printf '本次输出：%s\n正在保存运行参数（每个节点最多等待 20 秒）……\n' "$session_dir"
for node in controller_node fusion_node ble_hardware_bridge_node; do
  snapshot="$session_dir/parameters/runtime_${node}"
  if timeout 20s ros2 param dump "/$node" > "$snapshot.pending" 2> "$snapshot.stderr.txt" &&
      [[ -s "$snapshot.pending" ]]; then
    mv -- "$snapshot.pending" "$snapshot.yaml"
    printf '  已保存 /%s 参数\n' "$node"
  else
    mv -- "$snapshot.pending" "$snapshot.unavailable.txt"
    printf '  警告：/%s 参数快照失败（节点未启动、使用命名空间或服务超时）；继续录包。\n' "$node" >&2
  fi
done

printf 'record_requested_at: %s\n' "$(date --iso-8601=seconds)" >> "$session_dir/session_info.txt"
printf '\n即将录制 %s 个话题；确认 rosbag 开始录制后再测试。\n' "${#topics[@]}"
printf '按 Ctrl+C 正常结束，仅结束录包，不会停止轮椅。\n'
printf '查看结果：ros2 bag info "%s"\n\n' "$bag_dir"

# Replace the wrapper so Ctrl+C reaches rosbag directly and finalizes metadata.
# Topics with no publisher are discovered later; they may remain absent in the bag.
exec ros2 bag record --storage sqlite3 --output "$bag_dir" "${topics[@]}"
