#!/usr/bin/python
from math import *
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as matrix   # Library for matrix definition

global pos_f
pos_f = [0, 0]

class Node:
    def __init__(self, pos=[0, 0], father=None):
        global pos_f
        self.pos = pos
        self.father = father
        self.h = distance(self.pos, pos_f)

        if self.father == None:
            self.g = 0
        else:
            self.g = self.father.g + 1 # We add 1 to the value of path already traveled
        self.f = self.g + self.h       # Full path = Path already travelled + Path until the end

# Distance Function  a = x1 y1 b= x2 y2
def distance(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) 


class Planning(object):
  def __init__(self):
    self._tfBuffer = tf2_ros.Buffer()
    self._tfListener = tf2_ros.TransformListener(self._tfBuffer)
    self._pubPlan = rospy.Publisher('plan', Path, queue_size=1, latch=True)
    self._subGoal = rospy.Subscriber('goal', PoseStamped, self._handleGoal)

    #                                                                     1
    #                                                                     ↑
    # We define the map using the following codification :             8←   →2
    self.matrix = [[4,8,8,12,8,8,8,8,8,8,8,8,8,8],          #             ↓             
                   [4,2,2,6,3,2,6,2,2,2,2,2,4,1],           #             4
                   [4,1,0,4,1,0,2,2,4,0,0,0,4,1],
                   [4,1,0,4,1,0,0,0,4,0,4,8,8,9],
                   [4,1,0,4,9,8,12,8,8,0,4,2,2,1],
                   [4,1,0,6,3,2,6,2,3,2,4,1,0,1],
                   [4,1,0,4,1,0,4,0,1,0,4,1,0,1],
                   [4,1,0,4,1,0,4,0,1,0,4,1,0,1],
                   [4,1,8,12,9,8,12,8,9,8,12,9,8,9],
                   [6,2,2,6,3,2,6,2,3,2,2,3,2,1],
                   [4,0,0,4,1,0,4,0,1,0,0,0,0,1],
                   [2,2,2,2,3,2,2,2,3,2,2,2,2,1]]
                                     
                   
  def _handleGoal(self, msg):
    rospy.loginfo('New goal received ...')
    try:
      self.trans = self._tfBuffer.lookup_transform('world', 'robot14', rospy.Time())
    except tf2_ros.TransformException:
      rospy.logerr('No path is generated since TF from world to wmr is not available.')
      return
      
    # We call the function that give us the initial and final position
    self.findPos(msg)

    # We create 2 lists, one for the open cells,containing the cells that are still avaliable, and another one for closed cell,containing the
    # cells that the robot has already gone through
    self.open = []
    self.closed = []

    # We add initial node to closed list by appending it
    self.closed.append(self.InitialCell)

    # We add neighbours to the open list
    self.open += self.neighbours(self.InitialCell) # Only initial cell

    # We check is the objective is on the closed list
    while self.objective():
        self.lookFor()

    self.pathway = self.SendPathway()
           

  # Function that calculates goal and initial points as nodes in a global variable
  def findPos(self, msg):
    global pos_f
    xInitial = self.trans.transform.translation.x
    yInitial = self.trans.transform.translation.y

    # Initial Pose
    InitialCellX = int(xInitial // 0.15)
    InitialCellY = int(11-(yInitial // 0.15))
    self.InitialCell = Node([InitialCellX, InitialCellY])
    
    # Goal Pose
    xGoal = msg.pose.position.x
    yGoal = msg.pose.position.y
    
    GoalCellX = int(xGoal // 0.15)
    GoalCellY = int(11-(yGoal // 0.15))
    pos_f=[GoalCellX, GoalCellY]
    self.GoalCell = Node([GoalCellX, GoalCellY])

  # Retuns a list with the transitable neighbour nodes takking into account the road directions and green areas
  def neighbours(self, node):
      neighbours =[]

      # 0 is X and 1 is Y
      print self.matrix[node.pos[1]][node.pos[0]],node.pos
      # We will use binnary operator and to see which nodes we have to add to neighbours
      if (self.matrix[node.pos[1]][node.pos[0]]&8 != 0):
          neighbours.append(Node([node.pos[0]-1, node.pos[1]], node))    
      if (self.matrix[node.pos[1]][node.pos[0]]&4 != 0):
          neighbours.append(Node([node.pos[0], node.pos[1]+1], node))                
      if (self.matrix[node.pos[1]][node.pos[0]]&2 != 0):
          neighbours.append(Node([node.pos[0]+1, node.pos[1]], node))                
      if (self.matrix[node.pos[1]][node.pos[0]]&1 != 0):
          neighbours.append(Node([node.pos[0], node.pos[1]-1], node))                
      return neighbours

  # Switchs from the open list to the closed list the element with smaller F
  def minor_f(self):
      a = self.open[0]
      n = 0
      for i in range(1, len(self.open)):
          if self.open[i].f < a.f:
              a = self.open[i]
              n = i
      self.closed.append(self.open[n])
      del self.open[n]

  # Checks if a node is already in an list
  def in_list(self, node, lista):
      for i in range(len(lista)):
          if node.pos == lista[i].pos:
              return 1
      return 0

  # Manage the neighbours of the selected node
  def route(self):
      for i in range(len(self.nodes)):
          if self.in_list(self.nodes[i], self.closed):
              continue
          elif not self.in_list(self.nodes[i], self.open):
              self.open.append(self.nodes[i])
          else:
              if self.select.g+1 < self.nodes[i].g:
                  for j in range(len(self.open)):
                      if self.nodes[i].pos == self.open[j].pos:
                          del self.open[j]
                          self.open.append(self.nodes[i])
                          break

  # Analyze the last element of the closed list
  def lookFor(self):
      self.minor_f()
      self.select = self.closed[-1]
      self.nodes = self.neighbours(self.select)
      self.route()

  # Check if the objective is in the open list
  def objective(self):
      for i in range(len(self.open)):
          if self.GoalCell.pos == self.open[i].pos:
              return 0
      return 1

  # Return a list with the positions with the path to follow
  def SendPathway(self):
      for i in range(len(self.open)):               # Look for the goal position in open list in order to declare the objetive
          if self.GoalCell.pos == self.open[i].pos:
              objective = self.open[i]

      pathway = []
      while objective.father != None:
          pathway.append(objective.pos)
          objective = objective.father
      pathway.reverse()
      
      plan = Path()
      
      plan.header.frame_id = 'world'
      for p in pathway:
        r = PoseStamped()
        r.pose.position.x = p[0]*0.15+0.075
        r.pose.position.y = (11-p[1])*0.15+0.075
        r.pose.orientation.w = 1
        plan.poses.append(r)
      self._pubPlan.publish(plan)       # Message plan is published

      return pathway


if __name__ == '__main__':
  rospy.init_node('planning')
  planning = Planning()
  rospy.spin()
