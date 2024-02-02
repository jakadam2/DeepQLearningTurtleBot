#! /usr/bin/env python
from world import World
import rospy
from project.srv import Controller,ControllerRequest
from project.msg import State

N_FEATURES = 39
N_ACTIONS = 5

class TargetAchiever:

    action_space = [2.5,1.25,0,-1.25,-2.5]

    def __init__(self) -> None:
        rospy.init_node('target_achiever')
        self.world = World(0.4,0.4,5,5)
        rospy.wait_for_service('/Controller')
        self.model_server = rospy.ServiceProxy('/Controller',Controller)

    def achieve_target(self,x,y):
        current_state = self.world.reset_and_spawn(x,y,delete_target= False)     
        done = False
        while not done:
            alter_action = self.model_server(ControllerRequest(State(current_state.tolist())))
            current_state,done = self.world.eval_step(alter_action.action.action_nr)     
        self.world.reset_world()
        rospy.loginfo(f'Target: {x},{y} achieved')


if __name__ == '__main__':
    a = TargetAchiever()
    while not rospy.is_shutdown():
        x,y = input('Give coords to achieve: ').split(' ')
        x,y = float(x),float(y)
        a.achieve_target(x,y)