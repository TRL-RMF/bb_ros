# bb_ros

Source noetic
Add this file into your workspace/src 
build file
Source workspace

How to run the code:

1. roscore
2. ls /dev/tty* (to check which port you are on)
3. rosrun rosserial_python serial_node.py _port:=/dev/ttyACM1 (depends on what the port is, ususally it is ACM1 or ACM0
4. rosservice call /trolley_lifting_arm_srv "data: true" true for trolley to go up and false for trolley to go down

How it works:

rosservice call makes it move up and down. 

run rostopic echo /arm_status to check the arm status. Output will be 0 at bottom, 1 at top.

run rostopic echo /trolley_dock_status to check the docking status. Output will be 1 when it is docked and 0 when not docked.
