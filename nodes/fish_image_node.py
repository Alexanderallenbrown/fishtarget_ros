#!/usr/bin/env python

#Alex Brown
#2013

#currently, this node is in the process of being updated. Its purpose is to "measure" the lanes in a preview-sense. Currently, it does not use any map information, but eventually it will subscribe to the current "best" pose estimate, and use the map geometry to help estimate depth, etc. Right now the flat road assumption is key. There are a lot of commented debugging tools embedded here, that can be used if needed, but the idea now is to publish the lane marker feature (right lane at this writing) as a marker message, probably a line_strip or a points_list type. This way, you can see the relationship between the original image, the local locations of points perceived, and their global locations easily in RVIZ without messing around with costly CVwindows, etc. 

#remember, for this to work, the ROS TFs representing the relationship between car and world (/world frame to /car frame) and car to camera have to be set up and published properly. When this is achieved, ROS does all of the coordinate transformations for us in RVIZ.


import roslib
roslib.load_manifest('fish_target')
import sys
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from fish_target.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from numpy import pi
import math
import cv2
import tf

#std_msgs/Header header
#float32 arduino_time
#float32 sensorval_raw
#float32 sensorval_filtered
#float32 num_shots
#float32 trial_num
#float32 target_position
#float32 target_in
#float32 target_out

class target_image_overlay:

  def __init__(self):
    
    #this is how we get our image in to use openCV
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imagecallback,queue_size=1)#change this to proper name!
    self.target_sub = rospy.Subscriber("/fishtarget/targetinfo",TargetMsg,self.targetcallback,queue_size=1)
    self.image_pub = rospy.Publisher('/fishtarget/overlay_image',Image,queue_size=1)
    self.timenow = rospy.Time.now()
    self.arduinotime = 0
    self.sensorval_raw = 0
    self.sensorfal_filtered = 0
    self.num_shots = 0
    self.trial_num = 0
    self.target_in = 0
    self.target_out = 0



  def targetcallback(self,data):
    self.arduinotime = data.arduinotime
    self.sensorval_raw = data.sensorval_raw
    self.sensorfal_filtered = data.sensorfal_filtered
    self.num_shots = data.num_shots
    self.trial_num = data.trial_num
    self.target_in = data.target_in
    self.target_out = data.target_out

  #this function fires whenever a new image_raw is available. it is our "main loop"
  def imagecallback(self,data):
    font = cv2.FONT_HERSHEY_SIMPLEX
    try:
      frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    self.timenow = rospy.Time.now()
    rows,cols,depth = frame.shape

    cv2.putText(frame,'Time Since Last Trial: '+str(self.arduinotime)+' sec',(50,350),font,1,(0,0,255),2)
    cv2.putText(frame,'Trial Number: '+str(self.trial_num),(50,380),font,1,(0,0,255),2)
    cv2.putText(frame,'Sensor Value: '+str(self.sensorval_raw)+' counts',(50,440),font,1,(0,0,255),2)
    cv2.putText(frame,'Detected Shots: '+str(self.num_shots),(50,410),font,1,(0,0,255),2)
    cv2.putText(frame,'Sensor Median Diff: '+str(self.sensorfal_filtered),(50,470),font,1,(0,0,255),2)
    #cv2.imshow('test',frame)

    img_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
    img_out.header.stamp = rospy.Time.now()
    try:
        self.image_pub.publish(img_out)
    except CvBridgeError as e:
        print(e)

def main(args):
  
  rospy.init_node('fish_image', anonymous=True)
  ic = target_image_overlay()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
