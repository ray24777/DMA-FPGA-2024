#!/bin/bash

# do dma loop back test, check file md5sum

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

data_dir=xfer_data_tmp
file_size=`ls -l $1 | awk '{print $5}'`
mkdir $data_dir

dd if=$1 of=./$data_dir/$1_tmp bs=$2 count=1
sync
echo "file size: "$file_size"   test size: "$2

for i in $(seq 1 $3)
do
    # change to loopback mode
    selectMode 3

    echo -n "[$i] "

    # exec tx and rx
    ./dma_from_device -d /dev/ANLOGIC-PCI0_c2h_0 -a 0 -s $2 -o 0 -f ./$data_dir/LOOP_BACK_RX_FILE_$1-$i &
    ./dma_to_device -d /dev/ANLOGIC-PCI0_h2c_0 -a 0 -s $2 -o 0 -f ./$data_dir/$1_tmp &
    wait
    
    # calc md5sum
    rx_file_md5=`md5sum ./$data_dir/LOOP_BACK_RX_FILE_$1-$i | awk '{printf $1}'`
    tx_file_md5=`md5sum ./$data_dir/$1_tmp | awk '{printf $1}'`

    if [ "$rx_file_md5" != "$tx_file_md5" ]; then
        echo "test failed... FILE"$i
	echo "rx md5: "$rx_file_md5
        echo "tx md5: "$tx_file_md5
        exit 1
    fi
done

rm -rf ./$data_dir

echo "test success"

