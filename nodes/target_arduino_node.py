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

   def loop(self):
      line = self.ser.readline()
      linestrip = line.strip('\r\n')
      linesplit = line.split()
      if len(linesplit)==7:
        #print shotslast, arduinonumshots,shotslast==(arduinonumshots+1)
        
        try:
            msg = TargetMsg()
            msg.header.stamp = rospy.Time.now()

            msg.arduinotime = float(linesplit[0])
            msg.signal_raw = float(linesplit[3])
            msg.num_shots = float(linesplit[5])
            msg.trial_num = float(linesplit[4])
            msg.sensorval_filtered = float(linesplit[6])
            self.pub.publish(msg)
            while len(line)>0:
              line = self.ser.readLine()

        except:
            print "OOPS! BAD LINE"
            #ef.write("problem  with serial line"+"\r\n")




   def joycallback(self,data):
      throttlecommand = data.axes[0]
      steercommand = data.axes[1]

      leftmotorcommand = int((throttlecommand+steercommand)*128-self.leftmotor_offset)#condition the joy command for forward/backward
      rightmotorcommand = int((throttlecommand-steercommand)*128-self.leftmotor_offset)#condition the joy command for forward/backward

      IRcommand = int(90+data.axes[2]*90)#this should give s 0-180
      self.ser.write(str(IRcommand)+' '+str(leftmotorcommand)+' '+str(rightmotorcommand)+'\r\n')
      print 'sent: '+str(IRcommand)+' '+str(leftmotorcommand)+' '+str(rightmotorcommand)+'\r\n'
      #now that we wrote that, let's publish a transform from baby bot's base_link to the IR_head frame
      br = tf.TransformBroadcaster();
      br.sendTransform((0,0,0.1),tf.transformations.quaternion_from_euler(0,0,IRcommand*3.14/180), rospy.Time.now(),"/IR_head","/base_link")


      myline = self.ser.readline()
      
      if len(myline)>0:
        myrangedata = myline.split()#strip off characters
        print "myrangedata of zero is "+str(myrangedata[0])
        myrangedata = float(myrangedata[0])
      else:
        rospy.logwarn('DID NOT RECEIVE ANYTHING BACK FROM ARDUINO!')

      #now we will put myrangedata into the range message
      self.IR_msg.header.stamp = rospy.Time.now()
      self.IR_msg.range=myrangedata
      self.pub.publish(self.IR_msg)
      
      
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