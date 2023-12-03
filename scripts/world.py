#! /usr/bin/env python
import rospy
from geometry_msgs.msg import  Pose,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest, GetWorldProperties
import os
import random

class World:

    def __init__(self,x,y,w,h) -> None:
        # here will be parameters of the world
        self._x = x
        self._y = y
        self._h = h
        self._w = w
        pass 

    def spawn_robot(x,y):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = '~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger'
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn('turtlebot3_burger', model_xml, '', spawn_pose, 'world')


    def spawn_target(self):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = '~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box'
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        x,y = random.uniform(self._x,self._x + self._w),random.uniform(self._y,self._y + self._h)
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn('target', model_xml, '', spawn_pose, 'world')
        

    def delete_target(self,model_name):
        try:
            rospy.wait_for_service('gazebo/delete_model')
            del_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            del_model(model_name)
        except:
            pass

    def get_laser_scan(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5).ranges
            except:
                pass

        return [((self._x + self._w) if str(r) == 'inf' else min(r, (self._x + self._w))) for i, r in enumerate(data) if not i % 10]

        
