#!/usr/bin/env bash

set -u

read -r -p "请输入运行时间（秒，默认 10）: " duration
duration=${duration:-10}

if [[ ! "$duration" =~ ^[1-9][0-9]*$ ]]; then
  printf '错误：运行时间必须是大于 0 的整数秒，例如 10。\n' >&2
  exit 2
fi

printf '启动保持测试，运行 %s 秒...\n' "$duration"

timeout --foreground --signal=INT --kill-after=5s "${duration}s" \
  ros2 launch rebotarm_bringup rebotarm_hardware.launch.py \
    allow_motor_enable:=true \
    hold_only:=true \
    motor_enable_mask:=0,0,0,0,1,1 \
    start_arm_controller:=false &
launch_pid=$!

elapsed=0
while kill -0 "$launch_pid" 2>/dev/null; do
  printf '[计时] 已运行 %d / %d 秒\n' "$elapsed" "$duration"
  sleep 1
  ((elapsed += 1))
done

wait "$launch_pid"

status=$?
printf '\n保持测试结束：运行时间 %s 秒，退出状态 %d。\n' "$duration" "$status"
printf '正在强制失能 joint5、joint6 并验证反馈...\n'
if ros2 run rs_motor_sdk rs_motor_disable --execute can0 5 6; then
  printf '失能结果：joint5、joint6 均已确认 Disabled。\n'
else
  printf '失能结果：反馈验证失败，请立即检查电机状态。\n' >&2
  exit 1
fi

# timeout returns 124 when it stopped the command at the requested duration.
if [[ "$status" -eq 124 || "$status" -eq 130 || "$status" -eq 143 ]]; then
  printf '结果：已按设定时间自动退出。\n'
  exit 0
elif [[ "$status" -eq 0 ]]; then
  printf '结果：ROS 启动程序正常退出。\n'
else
  printf '结果：程序异常退出，请检查上面的 ROS 输出。\n'
fi

exit "$status"
