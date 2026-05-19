#!/bin/bash
# ────────────────────────────────────────────────────────────
# pigpiod_start.sh — 启动 pigpiod 守护进程（如果尚未运行）
# 在 launch 文件中通过 launch-prefix="sudo -E" 调用
# ────────────────────────────────────────────────────────────
set -e

if pidof pigpiod > /dev/null 2>&1; then
    echo "[pigpiod_start] pigpiod 已在运行 (PID: $(pidof pigpiod))"
    # 保持脚本运行，否则 launch 会认为节点退出
    exec tail -f /dev/null
else
    echo "[pigpiod_start] 启动 pigpiod..."
    pigpiod -s 2 -t 0  # -s 2: 采样率2μs, -t 0: 无PWM时钟限制
    sleep 1
    if pgrep pigpiod > /dev/null 2>&1; then
        echo "[pigpiod_start] pigpiod 启动成功 (PID: $(pgrep pigpiod))"
    else
        echo "[pigpiod_start] ERROR: pigpiod 启动失败！"
        exit 1
    fi
    exec tail -f /dev/null
fi
