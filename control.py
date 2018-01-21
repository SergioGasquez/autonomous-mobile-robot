#!/usr/bin/python

# Import libraries
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from time import sleep              # Library for implementing stops signs



class Control(object):

    
  def __init__(self):
    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
    self._pubCmdVel = rospy.Publisher('cmd_vel', Twist, queue_size=1)   # Publication for sending velocities to the robot
    self._subPlan = rospy.Subscriber('plan', Path, self._handlePlan)    # Subscription for plan in order to recibe comands from plannyng.py
    
    # Initialize variables
    self.k=None           # Index of the point that we are triying to reach
    self.v=0.3            # Linear Velocity
    self.validPath = 0    # Boolean used verify if the path is correct and to keep the program waiting
    
  def _handlePlan(self, msg):
    rospy.loginfo('New path with %d poses received.' % len(msg.poses))
    # We obtain the points to reach for the whole trajectory,appending them into arrays
    self.xRef=[]; self.yRef=[]; self.zRef=[]; self.wRef=[];
    self.length=len(msg.poses);         # Number of points in the recived path
    for i in range(0,self.length): 
      self.xRef.append(msg.poses[i].pose.position.x)        
      self.yRef.append(msg.poses[i].pose.position.y)
      self.zRef.append(msg.poses[i].pose.orientation.z)
      self.wRef.append(msg.poses[i].pose.orientation.w)
      self.phiRef=2*atan(self.zRef[i]/self.wRef[i])
      
    self.validPath = 1                  # Change the boolean since the recived path is correct
        
  def spin(self):elf
    self.aux=0                          # Boolean that checks if we have stopped already in a cell
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
      try:
        trans = self._tfBuffer.lookup_transform('world', 'robot14', rospy.Time())
      except tf2_ros.TransformException:
        rospy.logwarn('TF from world to wmr is not available.')
        rospy.sleep(rospy.Duration(1.0))
        continue
        
      # We obtain the position of the robot
      self.x=(trans.transform.translation.x)
      self.y=(trans.transform.translation.y)
      self.z=(trans.transform.rotation.z)
      self.w=(trans.transform.rotation.w)
      self.phi=2*atan2(self.z,self.w)   # Phi must be between [pi,-pi]
      # Transform the position into cells and into integers
      x=self.x // 0.15          
      y=self.y // 0.15
      x=int(x)
      y=int(y)

      # If the path is not valid,the robot won't move
      if self.validPath == 0:
          msg = Twist()
          msg.angular.z = 0
          msg.linear.x = 0
          self._pubCmdVel.publish(msg)
          rate.sleep()
          continue
        
      if self.k is None:
        self.k=0
      # We use the formula to obtain the angle that we should take to reach the goal
      phiTarget=atan2((self.yRef[self.k]-self.y),(self.xRef[self.k]-self.x))
        
      k2=1    # Angular velocity gain
      
      
      ePhi=phiTarget-self.phi       # Difference between robots angle and goal position
      # ePhi must be between [pi,-pi]
      if ePhi > pi:
        ePhi=ePhi-2*pi
      if ePhi < -pi:
        ePhi=ePhi+2*pi
      
      # Definition of velocities   
      self.Omega=k2*(ePhi)                   
      self.v=0.1*cos(ePhi)**2       # We multiply the speed with the cos² in order to reduce the speed when it goes through a corner
      

      
      D=sqrt((self.xRef[self.k]-self.x)**2 + (self.yRef[self.k]-self.y)**2)     # Distance between the robot and the actual goal point
      if (D<0.1):                   # We set the threshold at 10cms 
          self.k=self.k+1           # We face the next goal point
          if (self.length==self.k): # We check if we are in the last point
              self.v=0
              self.Omega=0
              self.k=0
              self.validPath = 0
              
              
              
      # Print Distance and velocities        
      print('Distance:%f'%D)
      print('v:%f'%self.v)
      print('w:%f'%self.w)
      
      # Stop signs map in form of bolean matrix   
      stop =      [[0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
                   [0,0,0,0,0,0,0,0,0,0,0,0,0,0], 
                   [0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                   [0,0,0,1,0,0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,1,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0,1,0,0,0,1],
                   [0,0,0,0,1,0,0,0,1,0,0,0,0,0],
                   [0,0,0,1,0,0,1,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0,0,0,0,1,0],
                   [0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,1,0,0,0,1,0,0,0,0,1],
                   [0,0,1,0,0,1,0,0,0,0,0,0,0,0]]

      
          
      if (stop[11-y][x]==1 and self.aux==0):      # We check if we are in a stop sign and if have already stopped in this cell
          # We make a 1 second stop by publishing the message with all the velocities set at 0 and change the aux boolean
          msg = Twist()
          msg.angular.z = 0
          msg.linear.x = 0  
          self._pubCmdVel.publish(msg)
          self.aux=1
          sleep(1) 
          # We publish the message with the velocities that we had before stopping
          msg = Twist()
          msg.angular.z = self.Omega
          msg.linear.x = self.v
          self._pubCmdVel.publish(msg)
      elif(stop[11-y][x]==0):  # If we are not in a stop cell ,the aux boolean is 0
          self.aux=0  
      # Message containing velocities is sent to the robot
      msg = Twist()
      msg.angular.z = self.Omega
      msg.linear.x = self.v
      self._pubCmdVel.publish(msg)

      rate.sleep()
        

if __name__ == '__main__':
  rospy.init_node('control')
  control = Control()
  control.spin()
