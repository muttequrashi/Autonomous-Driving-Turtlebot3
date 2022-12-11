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

## Connecting to Turtlebot3

Connecting to turtlebot over ssh with password napelturbot

      ssh ubuntu@192.168.0.200
      
 This is very important to bringup the turtlebot before running any node related to camera to robot control. 
 
Bring Up Turtlebot on PI.

     roslaunch turtlebot3_bringup turtlebot3_robot.launch 

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

## ROS Package
Overall structure of the ros packge is shown below.

![ROS Package](images/tree.png)

As seen in the above picture we have 4 different launch files [extrinsic_calibration.launch](launch/extrinsic_calibration.launch), [intrinsic_calibration.launch](launch/intrinsic_calibration.launch), [controller.launch](launch/controller.launch) and [lane_detection.launch](launch/lane_detection.launch).

First two the extrinsic_calibration.launch and intrinsic_calibration.launch are responsible to subscribe to camera topic "camera/image/compressed" and then applying the projection and compensation and then publishing it on topics "/camera/image_projected/compressed/", "/camera/image_projected_compensated/compressed". 

We will use these topics to get the projected camera image and perform lane detection. The lane_detection.launch file is running the [lane_detection.py](scripts/lane_detection.py) which is subscribbing to /camera/image_projected/compressed/ performs lane detection and then publishs the thresholding results on "/camera/mask_lane_detected/compressed" and the final lane detection on "/camera/midlane_detected/compressed" and the detected midpoint location in pixels on "/detect/lane". 

The [controller.launch](launch/controller.launch) runs the [controller.py](scripts/controller.py) which subscribes to "/detect/lane", "/control/max_vel"
and publishes the required velocity on turtlebot velocity topic "/cmd_vel".
Till now we have tested the lane tracking with a PD controller.

## Lane Detection

For lane detection we are using two different types of threshold values depending on the maximum intensity of pixel in the image. This was implemented to overcome the issue of low light in the tunnel. Our lane detection is mainly based on binary thresholding of the projected image and then creating a histogram. We have 3 different conditions to check if the robot should turn right, left or it should move straight. We are doing that by checking the value of pixels in the right and left half of the histogram. if both halfs have the line we publish the value of center to the "/detect/lane" topic that means robot should move straight. 
Our main idea of lane detection was inspired by [The Ultimate Guide to Real-Time Lane Detection Using OpenCV][2]. We tried implementing this code with our ROS packge but the results were not very great so due to time contraints we moved to binary thresholding and using some part of the code from [The Ultimate Guide to Real-Time Lane Detection Using OpenCV][2]. The files for this implementation are [lane_detection_1.py](scripts/lane_detection_1.py) and [edge_detection.py](scripts/edge_detection.py)

**Algorithm Steps

      Thresholding
      Apply Perspective Transformation to Get a Bird’s Eye View
      Identify Lane Line Pixels
      Set Sliding Windows for White Pixel Detection
      Fill in the Lane Line
      Overlay Lane Lines on Original Image
      Calculate Lane Line Curvature
      Calculate the Center Offset
      Display Final Image
      Publish Center to controller

The results were good on the straight path for a short distance but for longer distance it was not very satifactory.
Some results from the above mentioned algorythem 
<p float="left">
  <img src="images/masked_image.png" width="100" />
  <img src="images/midlane.png" width="100" /> 
  <img src="images/image.png" width="100" />
</p>


To Run the code we need to do following steps on remote PC and Turtlebot.

- Run ros master on PC. 

      roscore

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
[2]: https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/
