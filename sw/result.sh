#!/bin/bash

# 检查是否提供了hex文件名参数
if [ $# -ne 1 ]; then
  echo "用法: $0 <基地址偏移>"
  exit 1
fi

# 确保你使用的是 Bash 4.0 或更高版本
if [[ "${BASH_VERSINFO:-0}" -lt 4 ]]; then
  echo "This script requires Bash 4.0 or newer." >&2
  exit 1
fi

addr_off="$1"
# 基地址
base_address=$((0x8f880000+"$addr_off"))

# 地址数量
num_addresses=10

# 声明一个数组来存储读取到的值和对应的地址
declare -A values_and_addresses  # 使用关联数组来同时存储值和地址
 
# 遍历每个地址，读取数据并存储到数组中
for ((i=0; i<$num_addresses; i++)); do
  # 计算当前地址
  addr=$((base_address + i * 4))
  
  # 读取数据（假设devmem2输出格式固定，且我们关心的是第三行的内容）
  # 注意：这里我们假设devmem2的输出中没有其他干扰信息，且格式始终如一
  output=$(./devmem2 "$addr" 2>/dev/null)  # 将错误输出重定向到/dev/null，避免干扰
  
  # 提取第三行（Bash数组索引从0开始，所以第三行是索引2）
  # 并使用awk来提取括号前的地址部分
  if [[ -n "$output" ]]; then
    line=$(echo "$output" | sed -n '3p')  # 使用sed来打印第三行
    address_part=$(echo "$line" | awk -F'[(]' '{print $1}' | awk '{print $NF}')  # 使用awk来提取括号前的最后一个字段（应该是地址）
    value=$(echo "$line" | awk -F': ' '{print $NF}')  # 提取值部分（冒号后的内容）
    
    # 去除地址中的前导空格（如果有的话）
    address_part="${address_part##*( )}"
    
    # 将值和地址存储到关联数组中
    values_and_addresses["$address_part"]="$value"
  fi
done
 
# 初始化最大值、对应的地址和偏移量
max_value=""
max_address=""
offset=0
 
# 遍历关联数组，找到最大值和对应的地址
for address in "${!values_and_addresses[@]}"; do
  value="${values_and_addresses[$address]}"
  if [[ -z "$max_value" || "$(printf '%x\n' "$value" "$max_value" | sort -n | head -n 1)" != "$max_value" ]]; then
    max_value="$value"
    max_address="$address"
  fi
done
 
offset=$((max_address-base_address))
# 将偏移量除以 4 并输出结果
offset_div_4=$((offset / 4))
# 但是，由于我们只需要知道十进制结果，所以下面的转换是可选的
# offset_hex_formatted=$(printf "0x%x\n" "$offset_hex")
 
# 输出结果
echo "The maximum value is: $max_value at address: $max_address"
echo "The result is: $offset_div_4"
