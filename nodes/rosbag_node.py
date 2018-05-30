#!/usr/bin/env python

import rospy
import subprocess
import os
import signal
import roslib
roslib.load_manifest('fish_target')
from fish_target.msg import *




class RosbagRecord:
    def __init__(self):
        self.record_script = rospy.get_param('~record_script')
        self.record_folder = rospy.get_param('~record_folder')
        rospy.on_shutdown(self.stop_recording_handler)
        self.recording = False


        # Wait for shutdown signal to close rosbag record
        #rospy.spin()
        self.target_sub = rospy.Subscriber("/fishtarget/targetinfo",TargetMsg,self.targetcallback,queue_size=1)

    def targetcallback(self,data):
        #print data.target_out
        #print "getting target data!"
        self.target_out = data.target_out
        if(self.target_out==1):
            if self.recording==False:
                # Start recording.
                print "starting record"
                # command = "source " + self.record_script
                # print command
                # self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=self.record_folder,
                #                           executable='/bin/bash')
                #TODO fix this!!!
                self.p = subprocess.Popen("rosbag record /fishtarget/overlay_image/compressed fishtarget/targetinfo", shell=True,cwd='/home/michael-brown/Desktop/NEW_FISH_BAGS')
                self.recording=True
        else:
            if self.recording==True:
                print "stopping record"
                self.stop_recording_handler()
                self.recording = False

    def terminate_ros_node(self, s):
        # Adapted from http://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
        list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, "List command returned %d" % retcode
        for str in list_output.split("\n"):
            if (str.startswith(s)):
                os.system("rosnode kill " + str)
                print str

    def stop_recording_handler(self):
        rospy.loginfo(rospy.get_name() + ' stop recording.')
        self.terminate_ros_node("/record")

if __name__ == '__main__':
    rospy.init_node('rosbag_node',anonymous=True)
    #rospy.loginfo(rospy.get_name() + ' start')

    # Go to class functions that do all the heavy lifting. Do error checking.
    rosbag_record = RosbagRecord()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"