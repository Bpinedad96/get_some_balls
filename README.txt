
1. roslaunch get_some_balls follow.launch 

2. rosrun rosserial_python serial_node.py /dev/ttyACM0

3. rostopic echo /servo_pos 


Launch python script to search ball
Initialize serial
POssition of servo is published too servo_pos


To callibrate use script located in scripts:

python callibrate.py --filter HSV --webcam

