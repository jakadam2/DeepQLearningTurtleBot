#! /usr/bin/env python
import rospy
from geometry_msgs.msg import  Pose,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState, SetModelStateRequest, GetWorldProperties
import os
import random
import math
import torch
from pathlib import Path


class World:

    def __init__(self,x,y,w,h,initx = 0,inity = 0,vel = 0.30) -> None:
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
        path = os.path.join(Path(__file__).parent.parent,'models','turtlebot3_burger')
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        spawn('burger', model_xml, '', spawn_pose, 'world')


    def spawn_target(self):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = os.path.join(Path(__file__).parent.parent,'models','goal_box')
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        while True:
            x,y = random.uniform(self._x,self._x + self._w),random.uniform(self._y,self._y + self._h)
            self._target_pos = (x,y)
            spawn_pose.position.x = x
            spawn_pose.position.y = y
            if math.dist((4,4),(x,y) ) >= 0.7 and math.dist((4,2),(x,y) ) >= 0.7 and math.dist((2,4),(x,y) ) >= 0.7 and math.dist((2,2),(x,y) ) >= 0.7:
                break
        print(f'Goal coords X:{x} Y:{y}')
        spawn('target', model_xml, '', spawn_pose, 'world')
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
        smsReq.model_state.pose.position.x = 1
        smsReq.model_state.pose.position.y = 1
        service(smsReq)

    
    def reset_world(self,delete_target = True):
        self.stop_robot()
        self.teleport_robot('turtlebot3_burger',self._initx,self._inity)
        if delete_target:
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
        return odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.orientation,odom.twist.twist.angular.z
    

    def target_position(self):
        return self._target_pos
        

    def target_info(self):
        posx,posy,orient,twist= self.robot_position()
        tarx,tary = self.target_position()
        dist = math.hypot(tarx - posx, tary - posy)
        angle = math.atan2(tary - posy, tarx - posx)
        yaw = math.atan2(+2.0 * (orient.w * orient.z + orient.x * orient.y), 1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z))
        heading = angle - yaw
        if heading > math.pi:
            heading -= 2 * math.pi
        elif heading < -math.pi:
            heading += 2 * math.pi
        return [twist, dist, heading]


    def make_state(self):
        return torch.tensor(self.get_laser_scan() + self.target_info(),dtype = torch.float32,device='cuda')


    def calc_reward(self,state): # crash
        if min(state[1:len(state) - 3] ) < 0.15 or state[0] < 0.25:
            print('Wall hit')
            return torch.tensor([-2000],dtype=torch.float32,device='cuda'), True  

        if state[len(state) - 2] < 0.25: # goal
            print('Goal reached')
            return torch.tensor([300],dtype=torch.float32,device='cuda'), True
        
        return torch.tensor([1.5 * math.cos(state[len(state) - 1] )],dtype=torch.float32,device='cuda'),False
        

    def step(self,action):
        self.turn_robot(action)
        state = self.make_state()
        reward,done = self.calc_reward(state)
        return state,reward,done
    

    def reset(self,delete_target = True):
        self.reset_world(delete_target)
        self.spawn_target()
        return self.make_state()


    def set_target(self,x,y):
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        path = os.path.join(Path(__file__).parent.parent,'models','goal_box')
        with open (path + '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read().replace('\n', '')
        spawn_pose = Pose()
        self._target_pos = (x,y)
        spawn_pose.position.x = x
        spawn_pose.position.y = y
        if not (math.dist((4,4),(x,y) ) >= 0.7 and math.dist((4,2),(x,y) ) >= 0.7 and math.dist((2,4),(x,y) ) >= 0.7 and math.dist((2,2),(x,y) ) >= 0.7):
            raise TypeError('Is inposibble to put a target in occupied place')
        print(f'Target coords X:{x} Y:{y}')
        spawn('target', model_xml, '', spawn_pose, 'world')


    def reset_and_spawn(self,x,y,delete_target = True):
        self.reset_world(delete_target)
        self.set_target(x,y)
        return self.make_state()
    

    def check_target(self,state):
        return state[len(state) - 2] < 0.22


    def eval_step(self,action):
        self.turn_robot(action)
        state = self.make_state()
        done = self.check_target(state)
        return state,done