#!/bin/bash

CONFIG_DIR="configs"
LOG_DIR="logs"

mkdir -p "$LOG_DIR"

for config_file in "$CONFIG_DIR"/config_track*.json; do
    config_name=$(basename "$config_file")
    name_no_ext="${config_name%.*}"

    echo ">> 실행 중: $config_name"
    python main.py "$config_file" > "$LOG_DIR/${name_no_ext}.log" 2>&1
    echo ">> 완료: $config_name"
    echo "--------------------------------------"
done

echo "=== 모든 시뮬레이션이 완료되었습니다 ==="
