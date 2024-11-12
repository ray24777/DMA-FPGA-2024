#!/bin/bash

echo "                 _             _                      _                 ";
echo "     /\         | |           (_)                    | |                ";
echo "    /  \   _ __ | | ___   __ _ _  ___   ___  __ _  __| |_ __ ___   __ _ ";
echo "   / /\ \ | '_ \| |/ _ \ / _\` | |/ __| / __|/ _\` |/ _\` | '_ \` _ \ / _\` |";
echo "  / ____ \| | | | | (_) | (_| | | (__  \__ \ (_| | (_| | | | | | | (_| |";
echo " /_/    \_\_| |_|_|\___/ \__, |_|\___| |___/\__, |\__,_|_| |_| |_|\__,_|";
echo "                          __/ |              __/ |                      ";
echo "                         |___/              |___/                       ";
echo ""

function has_root() {
    if [[ $EUID -ne 0 ]]; then
        echo "Must use root/sudo to exec." 1>&2
		exit 1
    fi
}

function has_device() {
    region0=`lspci -d 1edb:abcd -vvv | grep "Region 0:"`
    if [[ -z $region0 ]]; then
        echo "Pcie card(1edb:abcd) not found." 1>&2
		exit 1
    fi
}

#update control reg c2h:1/h2c:2/loopback:3
function selectMode() {
    region0=`lspci -d 1edb:abcd -vvv | grep "Region 0:"`
    if [[ -z $region0 ]]; then
        echo "Pcie card(1edb:abcd) not found." 1>&2
		exit 1
    fi
    arr=(`echo ${region0#*at}`)
    region0_hex=${arr[0]}
    echo "region0 at $region0_hex"

    region0_num=`printf "%x" \'$region0_hex`

    loop_back_reg=$((16#${region0_hex}+524288))
    loop_back_reg_hex=0x`echo "obase=16;${loop_back_reg}"|bc`

    ./devmem2 $loop_back_reg_hex w $1
}

has_root
has_device

cat << EOF
********please enter your choise:(1-6)****
(1) Performance test for c2h.
(2) Performance test for h2c.
(3) One-way transmission for c2h.
(4) One-way transmission for h2c.
(5) Loopback test.
(6) Exit Menu.
EOF
read -p "Now select the top option to: " input
case $input in

1) 
read -p "Enter target channel: " channel
read -p "Enter single descripter size: " desc_sz
selectMode 1
./performance -d /dev/ANLOGIC-PCI0_c2h_$channel -s $desc_sz;;

2) read -p "Enter target channel: " channel
read -p "Enter single descripter size: " desc_sz
selectMode 2
./performance -d /dev/ANLOGIC-PCI0_h2c_$channel -s $desc_sz;;

3) read -p "Enter target channel: " channel
read -p "Enter transfer total size: " xfer_total_sz
read -p "Enter a file name for store recive data: " file
selectMode 1
./dma_from_device -d /dev/ANLOGIC-PCI0_c2h_$channel -v -s $xfer_total_sz -f $file;;

4) read -p "Enter target channel: " channel
read -p "Enter a file name for transfer data source: " file
read -p "Enter transfer total size(default file size, input must smaller than select file size): " xfer_total_sz
[ ! $xfer_total_sz ] && xfer_total_sz=`ls -l $file | awk '{print $5}'`
selectMode 2
./dma_to_device -d /dev/ANLOGIC-PCI0_h2c_$channel -s $xfer_total_sz -f $file;;

5) read -p "Enter a file name for transfer data source: " file
read -p "Enter transfer data size(default file size, input must smaller than select file size): " xfer_sz
[ ! $xfer_sz ] && xfer_sz=`ls -l $file | awk '{print $5}'`
read -p "Enter loopback test count(default 1): " xfer_cnt
[ ! $xfer_cnt ] && xfer_cnt=1
selectMode 3
./dma_loop_back_test.sh $file $xfer_sz $xfer_cnt;;

esac

