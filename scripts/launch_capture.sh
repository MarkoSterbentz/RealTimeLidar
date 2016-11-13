#!/bin/bash
#### Description: Simple script for launching the LiDAR capture program on the Raspberry Pi.
#### Written by:  Marko Sterbentz - stermark@isu.edu


# Set up variables
NETCTL_PROFILE=marko-N56VJ
INTERFACE=eth0
IP_ADDRESS=192.168.1.70/24
BROADCAST_ADDRESS=192.168.1.255
GATEWAY_ADDRESS=192.168.1.3

# Launch the netctl profile for connecting to the laptop's ad-hoc network
sudo netctl start $NETCTL_PROFILE

# Launch the ethernet connection to the Velodyne VLP-16
sudo ip link set $INTERFACE up
sudo ip addr add $IP_ADDRESS broadcast $BROADCAST_ADDRESS dev $INTERFACE
sudo ip route add default via $GATEWAY_ADDRESS

# Start LiDAR capture application
sudo ~/RealTimeLidar/build/source/capture/capture -s