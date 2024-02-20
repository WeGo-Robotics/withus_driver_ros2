# withus_driver_ros2

### $ git clone https://github.com/sreejithsreeni/withus_driver_ros2.git
### $ cd withus_driver_ros2
### $ colcon build
### $ source install/setup.bash
### $ pip3 install pyqt5

#### To enable the CAN communication Interface
### $ ros2 run withus_control can_bringup.sh

#### Enable the Driver 
### $ ros2 launch withus_control withus_control.launch.py

#### Run using ros commands
#### $ros2 topic pub /cmd _vel geometry_msgs/msg/Twist “linear:
x:3.0
y:0.0
z:0.0
angular:
x:0.0
y:0.0
z:20.0” 


##### ----> x - km/hr
##### ----> angular z (steering) - degree


#### For UI control 

### $ cd ~/withus_ws/src
	 	 	 	
### $ python3 pyqt_control.py
![Screenshot from 2024-02-20 17-21-42](https://github.com/sreejithsreeni/withus_driver_ros2/assets/107530509/1e1bc3b2-b4d6-4330-95da-a5c572d77185)

By Moving the speed and steering button, we can move the robot.



