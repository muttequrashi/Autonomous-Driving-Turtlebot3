# Self-Driving Turtlebot3 Using Camera based Lane Tracking



![Cover](images/cover.png)


Till now we have tested the lane tracking with a PD controller.

To Run the code we need to do following steps on remote PC and Turtlebot.

- Run ros master on PC. 

      roscore
- Connecting to turtlebot over ssh.

      ssh ubuntu@192.168.0.200
-Bring Up Turtlebot on PI.

     roslaunch turtlebot3_bringup turtlebot3_robot.launch 
-Start Capturing from Camera.

     roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
- On PC we have to run following files to start lane detection and tracking in seprate terminals.
         roslaunch group_4 extrinsic_calibration.launch
         roslaunch group_4 intrinsic_calibration.launch 
         roslaunch group_4 lane_detection.launch 
         roslaunch group_4 controller.launch 

Tunnel Issue is fixed now. 

Previous Trial Video.
https://www.veed.io/view/3ac05459-09aa-445f-a14c-4054755bf27d?panel=share

Complete Run Video.

https://www.veed.io/view/ee8fc177-8a91-421e-b204-14b9655aba3f?panel=share
