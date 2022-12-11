# Autonomous-Driving Turtlebot3 Using Camera based Lane Tracking

Supervised by: @renatojmsdh @duverneraphael @joako1991 
Submitted By: @muttequrashi @husein-loubani

![Cover](images/cover.png)

## Project Goal
Autonomous Driving of a Ground Differential Robot by Perception based on [Autorace Challenge][1]. Main task was to detect the lanes and make the robot follow lane and complete the mission. In the circuit provided robot has to go through a low light tunnel also.



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

[1]:https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving
