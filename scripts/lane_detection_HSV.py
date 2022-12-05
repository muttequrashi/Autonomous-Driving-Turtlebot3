#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge

class lane_following:
    def __init__(self):
        self.image_lane_sub = rospy.Subscriber('/camera/image_projected/compressed', CompressedImage, self.callback, queue_size = 1, buff_size=2**24)
        self.detect_midlane_pub = rospy.Publisher('/camera/midlane_detected/compressed', CompressedImage, queue_size = 1)
        self.detect_mask_lane_pub = rospy.Publisher('/camera/mask_lane_detected/compressed', CompressedImage, queue_size = 1)
        self.pub_lane = rospy.Publisher('/detect/lane', Float64, queue_size = 1)
        self.cvBridge = CvBridge()
        self.prev_time = 0
        self.left_lane_fit_bef = 0
        self.right_lane_fit_bef = 0
        self.log_status = None

    def callback(self, image_msg):
        np_arr = np.fromstring(image_msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # convert to HSV, since red and yellow are the lowest hue colors and come before green
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # filter image to remove noise or smoothing the image
        gray = cv2.medianBlur(gray,9)
        # gray = self.filter.update(gray)
        print(gray.max())
        if gray.max() < 190:
            # create a binary thresholded image on grayscale image between white and yellow
            thresh = cv2.inRange(gray, 150, 190)
        else:
            
            # create a binary thresholded image on grayscale image between white and yellow
            thresh = cv2.inRange(gray, 150, 200)
        
        warped = thresh

        histogram = np.sum(warped[warped.shape[0]//2:,:], axis=0)
        # Peak in the first half indicates the likely position of the left lane
        half_width = np.int(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:half_width])
        #second half indicates the likely position of the right lane
        rightx_base = np.argmax(histogram[half_width:]) + half_width

        # Choose the number of sliding windows
        nwindows = 10
        # Set height of windows
        window_height = np.int(warped.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []
        
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = warped.shape[0] - (window+1)*window_height
            win_y_high = warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
        
        # Concatenate the arrays of indices
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 

        # Fit a second order polynomial to each
        try:
            right_fit = np.polyfit(righty, rightx, 2)   
            self.right_lane_fit_bef = right_fit
        except:
            right_fit = self.right_lane_fit_bef

        try:
            
            left_fit = np.polyfit(lefty, leftx, 2)
            self.left_lane_fit_bef = left_fit
        except:
            left_fit = self.left_lane_fit_bef

        # Generate x and y values for plotting
        ploty = np.linspace(0, warped.shape[0]-1, warped.shape[0] )
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        except:
            pass

        try:
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except:
            pass
        print("Left Base ", leftx_base, " Right Base ",rightx_base)
        if leftx_base < 100 and rightx_base > 600:
            cx, cy, _, _ = getPerpCoord(right_fitx[300], 300, right_fitx[301], 301, 255)
            img = cv2.circle(img, (cx,cy), radius=10, color=(0, 0, 255), thickness=-1)
            right_lane=np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            final = cv2.polylines(img, np.int_([right_lane]), isClosed=False, color=(255, 255, 255), thickness=24)
            if not self.log_status == 'right':
                rospy.loginfo('Detect only right lane')
                # self.prev_time = time.time()
                self.log_status = 'right'

        elif   leftx_base > 100 and rightx_base < 600:
            cx, cy, _, _ = getPerpCoord(left_fitx[300], 300, left_fitx[301], 301, -255)
            #print('CX Left line', cx)
            img = cv2.circle(img, (cx,cy), radius=10, color=(0, 0, 255), thickness=-1)
            left_lane=np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            final = cv2.polylines(img, np.int_([left_lane]), isClosed=False, color=(0, 255, 255), thickness=24)
            if not self.log_status == 'left':
                rospy.loginfo('Detect only left lane')
                # self.prev_time = time.time()
                self.log_status = 'left'

        elif leftx_base > 100 and rightx_base > 700:
            cx = int(round(np.mean([left_fitx[300], right_fitx[300]], axis=0)))
            
            img = cv2.circle(img, (cx, 300), radius=10, color=(0, 0, 255), thickness=-1)
            left_lane=np.array([np.transpose(np.vstack([left_fitx, ploty]))])
            right_lane=np.array([np.transpose(np.vstack([right_fitx, ploty]))])
            final = cv2.polylines(img, np.int_([left_lane]), isClosed=False, color=(0, 255, 255), thickness=24)
            final = cv2.polylines(final, np.int_([right_lane]), isClosed=False, color=(255, 255, 255), thickness=24)
            if not self.log_status == 'both':
                rospy.loginfo('Detect both lanes')
                # self.prev_time = time.time()
                self.log_status = 'both'

        else:
            cx = 467
            final = img
            if not self.log_status == None:
                rospy.loginfo('There is no lane')
                # self.prev_time = time.time()
                self.log_status = None
 
        # write lane center point pixel x to publish
        msg_desired_center = Float64()
        msg_desired_center.data = cx

        #publish
        self.pub_lane.publish(msg_desired_center)
        self.detect_mask_lane_pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(thresh, "jpg"))
        self.detect_midlane_pub.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, "jpg"))

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def getPerpCoord(aX, aY, bX, bY, length):
    vX = bX-aX
    vY = bY-aY
    #print(str(vX)+" "+str(vY))
    if(vX == 0 or vY == 0):
        return 0, 0, 0, 0
    mag = np.sqrt(vX*vX + vY*vY)
    vX = vX / mag
    vY = vY / mag
    temp = vX
    vX = 0-vY
    vY = temp
    cX = bX + vX * length
    cY = bY + vY * length
    dX = bX - vX * length
    dY = bY - vY * length
    return int(cX), int(cY), int(dX), int(dY)

def main():
    rospy.init_node('lane_detection')
    rospy.loginfo('Lane detection is running')
    lane_following()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass