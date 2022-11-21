# 2022_roboticsProject_group4
# Lane Detection and Tracking using Turtlebot3 and Raspberry Pi Cam
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


Trial Run Video.
https://www.veed.io/embed/304b4bfd-df87-4698-babc-a62f01239409
