#!/usr/bin/env python

import roslib; roslib.load_manifest('fish_target')
import rospy
from fish_target.msg import TargetMsg
import tf
import serial
import sys
import datetime

class ArduinoTarget():


   def __init__(self):
      self.port = rospy.get_param('~port','/dev/ttyACM0')
      self.baud = rospy.get_param('~baud',115200)
      self.ser = serial.Serial(self.port, self.baud,timeout=1) #this initializes the serial object
      self.pub= rospy.Publisher("fishtarget/targetinfo",TargetMsg)
      #capture the current date and time
      d = datetime.datetime.now()
      dirname = "/home/michael-brown/Desktop/fishlogs/"
      self.fname = dirname+str(d.year)+"-"+str(d.month)+"-"+str(d.day)+"-"+str(d.hour)+"-"+str(d.minute)+"-"+str(d.second)+".txt"
      self.f = open(self.fname,'a')
      self.num_shots = 0
      self.last_shots = 0
      #main loop runs on a timer
      rospy.Timer(rospy.Duration(.005),self.loop,oneshot=False) #timer callback (math) allows filter to run at constant time
      #subscribers (inputs)
      #construct the file name for our text output file

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
            self.num_shots = msg.num_shots
            msg.trial_num = float(linesplit[4])
            msg.sensorval_filtered = float(linesplit[6])
            self.pub.publish(msg)
            #print msg
            #while len(line)>0:
            #  line = self.ser.readLine()

        except:
            print "OOPS! BAD LINE"
            #ef.write("problem  with serial line"+"\r\n")

      if(self.num_shots==(self.last_shots+1)):
        tstamp = datetime.datetime.now()
        self.f.write(str(tstamp.year)+"\t"+str(tstamp.month)+"\t"+str(tstamp.day)+"\t"+str(tstamp.hour)+"\t"+str(tstamp.minute)+"\t"+str(tstamp.second)+"."+str(tstamp.microsecond)+line)
        self.f.close()
        self.f = open(self.fname,'a')
        print "WRITING NEW LINE!!!"
        self.last_shots=self.num_shots





      
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