#!/bin/bash
#### Description: Simple script for launching the LiDAR capture program on the Raspberry Pi.
#### Written by:  Marko Sterbentz - stermark@isu.edu

# Set up user's argument
if [ $# -eq 0 ] ; then
	echo "No arguments supplied. Valid arguments are 'start' and 'stop'."
	exit 1
else
	# Store the first argument
	ARG=$1

	# Set up variables
	NETCTL_PROFILE=marko-N56VJ
	INTERFACE=eth0
	IP_ADDRESS=192.168.1.70/24
	BROADCAST_ADDRESS=192.168.1.255
	GATEWAY_ADDRESS=192.168.1.3

	if [ $ARG = "start" ] ; then
		# Launch the netctl profile for connecting to the laptop's ad-hoc network
		sudo netctl start $NETCTL_PROFILE

		# Launch the ethernet connection to the Velodyne VLP-16
		sudo ip link set $INTERFACE up
		sudo ip addr add $IP_ADDRESS broadcast $BROADCAST_ADDRESS dev $INTERFACE
		sudo ip route add default via $GATEWAY_ADDRESS

		# Start LiDAR capture application
		sudo ~/RealTimeLidar/build/source/capture/capture -s

	else
		if [ $ARG = "stop" ] ; then
			# Stop the netctl profile that was connecting to the laptop's ad-hoc network
			sudo netctl stop $NETCTL_PROFILE

			# Stop the ethernet connection that was connecting to the Velodyne VLP-16
			sudo ip addr flush dev $INTERFACE
			sudo ip route flush dev $INTERFACE
			sudo ip link set $INTERFACE down

			# Restart the default ethernet internet connection
			sudo netctl start ethernet-dhcp
		else
			echo "'"$ARG"' is not a valid argument."
		fi
	fi
fi