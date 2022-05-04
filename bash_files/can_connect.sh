sudo ip link set dev can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set dev can0 up
sudo ifconfig can0 txqueuelen 20
