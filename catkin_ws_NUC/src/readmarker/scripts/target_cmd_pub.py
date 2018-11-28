#!/usr/bin/env python

import rospy
from readmarker.msg import markerXY                             #Custom msg for marker locationz
from geometry_msgs.msg import Twist                             #Msg for cmd_vel, contains linear/angular xyz velocity
import math                                                     #Include isnan function


class TargetCmdPub:
    def __init__(self):
        rospy.init_node('target_cmd_pub')                       #Initailize the node named as 'target_cmd_pub'
        self.targetXY_sub = rospy.Subscriber('/markerXY', markerXY, self.callback)      #Declare subscriber
        self.target_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)        #Declare publisher

    def callback(self,msg):
        cmd = Twist()                                           #Initialize command velocity
        if not any(map(math.isnan,msg.marker1X)):               #If received data is not nan value (detection success)
            cmd.linear.x = 0.2                                  #Set forward velocity 0.2 m/s
            cnt_marker1 = sum(msg.marker1X)/len(msg.marker1X)   #Get the mean value of x location of markers
            if cnt_marker1 <= msg.width/2:                      #If the value is on the left of image center
                cmd.angular.z = -0.1                            #Rotate -0.1 rad/sec (anti-clockwise)
            else:
                cmd.angular.z = 0.1                             #Otherwise, rotate 0.1 rad/sec (clockwise)
        elif not any(map(math.isnan,msg.marker2X)):             #Below is the case of 2, which is the same as 1
            cmd.linear.x = 0.2
            cnt_marker2 = sum(msg.marker2X)/len(msg.marker2X)
            if cnt_marker2 <= msg.width/2:
                cmd.angular.z = -0.1
            else:
                cmd.angular.z = 0.1
        self.target_cmd_pub.publish(cmd)                        #Publish the 'cmd_vel' topic
            

def main():
    target_cmd_pub = TargetCmdPub()                             #Initailize the instance of class TargetCmdPub
    rate = rospy.Rate(5)                                        #5Hz loop rate
    try:
        rospy.spin()                                            #Keep python from exiting
        rate.sleep()
    except KeyboardInterrupt:                                   #For Ctrl-C execution
        print("Shutting Down")
            

if __name__ == '__main__':
    main()
