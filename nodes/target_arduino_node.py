#!/usr/bin/env python

import roslib; roslib.load_manifest('fish_target')
import rospy
from fish_target.msg import TargetMsg
import tf
import serial
import sys

class ArduinoTarget():


   def __init__(self):
      self.port = rospy.get_param('~port','/dev/ttyACM0')
      self.baud = rospy.get_param('~baud',115200)
      self.ser = serial.Serial(self.port, self.baud,timeout=1) #this initializes the serial object
      self.pub= rospy.Publisher("fishtarget/targetinfo",TargetMsg)
      #main loop runs on a timer
      rospy.Timer(rospy.Duration(.005),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
      #subscribers (inputs)

   def loop(self,event):
      line = self.ser.readline()
      #print line
      linestrip = line.strip('\r\n')
      linesplit = line.split()
      if len(linesplit)==7:
        #print shotslast, arduinonumshots,shotslast==(arduinonumshots+1)
        
        try:
            msg = TargetMsg()
            msg.header.stamp = rospy.Time.now()
            msg.target_out = float(linesplit[2])
            msg.target_in = float(linesplit[1])
            msg.arduino_time = float(linesplit[0])
            msg.sensorval_raw = float(linesplit[3])
            msg.num_shots = float(linesplit[5])
            msg.trial_num = float(linesplit[4])
            msg.sensorval_filtered = float(linesplit[6])
            self.pub.publish(msg)
            #print msg
            #while len(line)>0:
            #  line = self.ser.readLine()

        except:
            print "OOPS! BAD LINE"
            #ef.write("problem  with serial line"+"\r\n")





      
#main function
def main(args):
  rospy.init_node('arduino_encoder_reader_node', anonymous=True)
  encreader = ArduinoTarget()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main(sys.argv) 