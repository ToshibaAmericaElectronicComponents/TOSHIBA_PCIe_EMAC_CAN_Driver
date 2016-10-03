#A Simple shell script to make network interface up
#20/May/2016
###############################################################################
clear
if [ $(id -u) != "0" ]; then #checking whehter the user is superuser or not.
    echo "You must be the superuser to run this script" >&2
    exit 1
fi

if [ $# -eq 0 ]
  then
    echo ""
    echo "Usage:"
    echo "    Enter loopback to be on or off"
    echo ""
    echo ""
    exit 1
fi

LOOP_MODE=$1

if [ "$LOOP_MODE" = "on" ];  then
	echo "Local Loopback is on"
elif [ "$LOOP_MODE" = "off" ];  then
    echo "Local Loopback is off"
else
    echo "argument of "loopback" must be "on" or "off""
	exit 1
fi

echo ""
echo ""
ip link set can0 up type can bitrate 1000000 sample-point 0.75 dbitrate 2500000 dsample-point 0.75 dsjw 4 loopback $LOOP_MODE fd on 
if [ $? -eq 0 ]	#Checking successful execution of previous command
  then
    echo "CAN0 up Success"
    break
  else
    echo "CAN0 up Failure"
    break
fi

echo ""
echo ""
ip -details link show can0

echo ""
echo ""
ip link set can1 up type can bitrate 1000000 sample-point 0.75 dbitrate 2500000 dsample-point 0.75 dsjw 4 loopback $LOOP_MODE fd on
if [ $? -eq 0 ] #Checking successful execution of previous command
  then
    echo "CAN1 up Success"
    break
  else
    echo "CAN1 up Failure"
    break
fi

echo ""
echo ""
ip -details link show can1

echo ""
echo ""
ip link set can2 up type can bitrate 1000000 sample-point 0.75 dbitrate 2500000 dsample-point 0.75 dsjw 4 loopback $LOOP_MODE fd on
if [ $? -eq 0 ] #Checking successful execution of previous command
  then
    echo "CAN2 up Success"
    break
  else
    echo "CAN2 up Failure"
    break
fi

echo ""
echo ""
ip -details link show can2

echo ""
echo ""
ip link set can3 up type can bitrate 1000000 sample-point 0.75 dbitrate 2500000 dsample-point 0.75 dsjw 4 loopback $LOOP_MODE fd on
if [ $? -eq 0 ] #Checking successful execution of previous command
  then
    echo "CAN3 up Success"
    break
  else
    echo "CAN3 up Failure"
    break
fi

echo ""
echo ""
ip -details link show can3

cd $CUR_PATH

echo ""
echo ""



