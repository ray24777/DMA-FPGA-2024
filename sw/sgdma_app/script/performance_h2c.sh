#!/bin/bash

region0=`lspci -d 1edb:abcd -vvv | grep "Region 0:"`
arr=(`echo ${region0#*at}`)
region0_hex=${arr[0]}
echo "region0 at $region0_hex"

region0_num=`printf "%x" \'$region0_hex`

loop_back_reg=$((16#${region0_hex}+524288))
loop_back_reg_hex=0x`echo "obase=16;${loop_back_reg}"|bc`

#update control reg c2h:1/h2c:2/loopback:3
./devmem2 $loop_back_reg_hex w 2

./performance -d /dev/ANLOGIC-PCI0_h2c_$1 -s $2
