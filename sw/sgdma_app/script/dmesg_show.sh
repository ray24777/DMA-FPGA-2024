while true 
do 
 dmesg >> ../test_log/dmesg_log.txt
 dmesg -c 
 sleep 1 
done 
