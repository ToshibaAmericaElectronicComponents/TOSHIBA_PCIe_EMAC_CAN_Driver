#A Simple shell script to unload PCIe CAN module and and network interface down
#20/May/2016
###############################################################################

echo ""
echo ""
ip link set can0 down 
if [ $? -eq 0 ]	#Checking successful execution of previous command
  then
    echo "CAN0 down Success"
    break
  else
    echo "CAN0 down Failure"
    break
fi

echo ""
echo ""
ip link set can1 down
if [ $? -eq 0 ] #Checking successful execution of previous command
  then
    echo "CAN1 down Success"
    break
  else
    echo "CAN1 down Failure"
    break
fi

echo ""
echo ""

ip link set can2 down 
if [ $? -eq 0 ]	#Checking successful execution of previous command
  then
    echo "CAN2 down Success"
    break
  else
    echo "CAN2 down Failure"
    break
fi

echo ""
echo ""

ip link set can3 down 
if [ $? -eq 0 ]	#Checking successful execution of previous command
  then
    echo "CAN3 down Success"
    break
  else
    echo "CAN3 down Failure"
    break
fi

echo ""
echo ""