#!/usr/bin/env python

#Alex Brown
#2013

import roslib
roslib.load_manifest('fish_target')
import sys
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from fish_target.msg import *
from sensor_msgs.msg import Image, CompressedImage
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
    self.compressed_image_pub = rospy.Publisher('/fishtarget/overlay_image/compressed',CompressedImage,queue_size=1)
    self.timenow = rospy.Time.now()
    self.arduino_time = 0
    self.sensorval_raw = 0
    self.sensorval_filtered = 0
    self.num_shots = 0
    self.trial_num = 0
    self.target_in = 0
    self.target_out = 0



  def targetcallback(self,data):
    self.arduino_time = data.arduino_time
    self.sensorval_raw = data.sensorval_raw
    self.sensorval_filtered = data.sensorval_filtered
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

    cv2.putText(frame,'Time Since Last Trial: '+str(self.arduino_time)+' sec',(50,350),font,1,(0,0,255),2)
    cv2.putText(frame,'Trial Number: '+str(self.trial_num),(50,380),font,1,(0,0,255),2)
    cv2.putText(frame,'Sensor Value: '+str(self.sensorval_raw)+' counts',(50,440),font,1,(0,0,255),2)
    cv2.putText(frame,'Detected Shots: '+str(self.num_shots),(50,410),font,1,(0,0,255),2)
    cv2.putText(frame,'Sensor Median Diff: '+str(self.sensorval_filtered),(50,470),font,1,(0,0,255),2)
    #cv2.imshow('test',frame)

    #### Create CompressedIamge ####
    cimg = CompressedImage()
    cimg.header.stamp = rospy.Time.now()
    cimg.format = "jpeg"
    cimg.data = np.array(cv2.imencode('.jpeg', frame)[1]).tostring()



    img_out = self.bridge.cv2_to_imgmsg(frame, "bgr8")
    img_out.header.stamp = rospy.Time.now()
    try:
      if self.target_out==1:
          self.image_pub.publish(img_out)
          self.compressed_image_pub.publish(cimg)
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
