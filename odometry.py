#!/usr/bin/python

# Import libraries
from wmr import Wmr
import rospy
import math
from geometry_msgs.msg import Twist
from PublisherOdom import PublisherOdom

class WmrRos(Wmr):

    def __init__(self):
        # Initialize variables
        self.a=0
        self.b=0
        self.x=0
        self.y=0
        self.phi=0
        Wmr.__init__(self)
        self._pubOdom=PublisherOdom(topic='odom',frame_id='luka/odom',child_frame_id='luka/wmr')
        rospy.Subscriber("cmd_vel", Twist, self._handleCmdVel)

    def _handleCmdVel(self, msg):
        self.setVel(msg.linear.x, msg.angular.z)

    def _handleEncoders(self,left,right):
        print('left = %d, right = %d A= %f B= %f' % (left, right,self.a,self.b))
        self.a=self.a+left                          # Storage of left wheel
        self.b=self.b+right                         # Storage of rigth wheel
        L = 0.068                                   # Length between wheels
        Calibrate=120000.0                          # Calibration constant
        
        w = (right-left)/(L*Calibrate)              # Angular velocity
        v = (right+left)/(2*Calibrate)              # Linear velocity
        print('Velocity= %f,Angular Vel= %f' % (v,w))
        
        # Euler Integration
        self.x=self.x+v*math.cos(self.phi)          # x(k+1)=x(k)+v(k)*Ts*cos(phi(k))
        self.y=self.y+v*math.sin(self.phi)          # y(k+1)=y(k)+v(k)*Ts*sin(phi(k))
        self.phi=self.phi+w                         # phi(k+1)=phi(k)+Ts*w
        print('X= %f,Y = %f ,Phi= %f' % (self.x,self.y,self.phi))
        # Send the message with robot pose and velocities
        self._pubOdom.publish(x=self.x,y=self.y,phi=self.phi,v=v,omega =w)

    
if __name__=='__main__':
    try:
        rospy.init_node('wmr', anonymous=True)

        with WmrRos() as robot:
            rate = rospy.Rate(20)
            while not rospy.is_shutdown():
                robot.updateSensors()
                rate.sleep()
    except KeyboardInterrupt:
        pass
