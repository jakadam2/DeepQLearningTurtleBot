#! /usr/bin/env python
import rospy
from geometry_msgs.msg import  Pose,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest, GetWorldProperties
import os
import random

class World:

    def __init__(self,x,y,w,h,initx = 0,inity = 0,vel = 0.15) -> None:
        # here will be parameters of the world
        self._x = x
        self._y = y
        self._h = h
        self._w = w
        self._initx = initx
        self._inity = inity
        self._vel_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self._vel = vel
        self._target_pos = None
        pass 

    def spawn_robot(self,x,y):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = '/home/adam/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger'
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn('burger', model_xml, '', spawn_pose, 'world')


    def spawn_target(self):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = '/home/adam/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box'
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        x,y = random.uniform(self._x,self._x + self._w),random.uniform(self._y,self._y + self._h)
        self._target_pos = (x,y)
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn('target', model_xml, '', spawn_pose, 'world')
        

    def delete_model(self,model_name):
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
                data = rospy.wait_for_message('/scan', LaserScan, timeout=5).ranges
            except:
                pass

        return [((self._x + self._w) if str(r) == 'inf' else min(r, (self._x + self._w))) for i, r in enumerate(data) if not i % 10]
    

    def teleport_robot(self,name,x,y):
        rospy.wait_for_service('gazebo/set_model_state')
        service = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
        smsReq = SetModelStateRequest()
        smsReq.model_state.model_name = name
        smsReq.model_state.pose.position.x = x
        smsReq.model_state.pose.position.y = y
        service(smsReq)

    
    def reset_world(self):
        self._vel_publisher.publish(Twist())
        self.teleport_robot('turtlebot3_burger',self._initx,self._inity)
        self.delete_model('target')


    def start_robot(self):
        start = Twist()
        start.linear.x = self._vel
        self._vel_publisher.publish(start)
        

    def stop_robot(self):
        self._vel_publisher.publish(Twist())


    def turn_robot(self,ang):
        turn = Twist()
        turn.linear.x = self._vel
        turn.angular.z = ang
        self._vel_publisher.publish(turn)


    def robot_position(self):
        odom = None
        while odom is None:
            try:
                odom = rospy.wait_for_message('odom', Odometry, timeout=5)
            except:
                pass
        return odom.pose.pose.position.x,odom.pose.pose.position.y
    

    def target_position(self):
        return self._target_pos
        
