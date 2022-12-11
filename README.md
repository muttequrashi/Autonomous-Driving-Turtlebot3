# Autonomous-Driving Turtlebot3 Using Camera based Lane Tracking

Supervised by: Renato Martins (@renatojmsdh) DUVERNE Raphael  (@duverneraphael) Joaquin Jorge Rodriguez (@joako1991) 
Submitted By: Mutte Ur Rehman (@muttequrashi) Hussein Loubani (@husein-loubani)

![Cover](images/cover.png)

## Project Goal
Autonomous Driving of a Ground Differential Robot by Perception based on [Autorace Challenge][1]. Main task was to detect the lanes and make the robot follow lane and complete the mission. In the circuit provided robot has to go through a low light tunnel also.

## Required Libraries and Packages to Start
      ros-noetic-image-transport 
      ros-noetic-cv-bridge 
      ros-noetic-vision-opencv 
      opencv 
      libopencv-dev 
      ros-noetic-image-proc
      Autorace package

## First Step Installing the Required Packages .
For this step we have followed the instructions provided on [emanual robotics][1].
Install the AutoRace 2020 meta package on Remote PC.

      cd ~/catkin_ws/src/
      git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_autorace_2020.git
      cd ~/catkin_ws && catkin_make
            
Install additional dependent packages on Remote PC.

      sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc

## Camera Calibaration

For Camera Callibration we have done the same proccedure as listed below. We used the autorace camera package to get the undistorted image so we can perform the lane detection.

Launch roscore on Remote PC.
      
      roscore
Trigger the camera on SBC.
      
      roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

### Intrinsic Camera Calibration
After launching the camera node from raspberry pi we can now run the following package on remote pc to do the camera calibration. We used the checkboard pattern to do this task. 

      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=calibration
      
One of the multiple pictures taken during calibration proccess is listed below.

![Calibartion](images/left-0033.png)

After completing the callibration we saved the data. By default calibrationdata.tar.gz is created at /tmp folder on remote pc. This compressed folder contain the sample images as well as the callibration data in a file named ost.yaml. For using the Autorace Camera package we copied the calibration file data and pasted into the file present in the Autorace Race package named [camerav2_320x240_30fps.yaml](/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml) 

### Extrinsic Camera Callibration

Open a new terminal and launch the intrinsic camera calibration node.
      
      roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch
Open a new terminal and launch the extrinsic camera calibration node.

      roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration

Now to set the region of interest and the warping parameters for projected image we can run the following command in new terminal.
      
      rosrun rqt_reconfigure rqt_reconfigure

After the extrinsic callibration the results are stored in  turtlebot3_autorace_camera/calibration/extrinsic_calibration/ having two files compensation.yaml and projection.yaml. We copied those files to our package so we can use them to republish the topic coming from raspberry pi after applying the desired projection and compensation. These file are located at [Extrinsic Calibration Files](/calibration/extrinsic_calibration)



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

