#!/bin/bash

# 检查是否提供了hex文件名参数
if [ $# -ne 2 ]; then
  echo "用法: $0 <hex文件名> <基地址偏移>"
  exit 1
fi

# 获取hex文件名参数
HEX_FILE="$1"
addr_off="$2"
# 起始地址
CURRENT_ADDR=$((0x8f880000+"$addr_off"))

# 检查文件是否存在
if [ ! -f "$HEX_FILE" ]; then
  echo "文件 $HEX_FILE 不存在!"
  exit 1
fi

# 逐行读取文件内容并写入地址
LINE_NUM=1 # 初始化行号（从1开始，因为地址从起始地址开始）

while IFS= read -r line || [ -n "$line" ]; do
  # 去除行首和行尾的空白字符（包括换行符）
  line=$(echo "$line" | xargs)

  # 检查行是否为空或不是有效的16进制数（这里简单检查长度是否为6，即24位）
  if [ -z "$line" ] || ! [[ "$line" =~ ^[0-9a-fA-F]{6}$ ]]; then
    echo "警告: 无效的数据行: $line，跳过写入"
    LINE_NUM=$((LINE_NUM + 1)) # 递增行号，即使跳过了写入
    continue
  fi

  # 使用devmem2写入数据
  # echo "写入地址 $CURRENT_ADDR 数据 $line"
  ./devmem2 "$CURRENT_ADDR" w "$line"

  # 递增地址（每次递增4）
  CURRENT_ADDR=$((CURRENT_ADDR + 4))

  # 递增行号
  LINE_NUM=$((LINE_NUM + 1))

done < "$HEX_FILE"
