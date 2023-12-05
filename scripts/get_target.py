#! /usr/bin/env python
from world import World
from model import DQN
import torch
import rospy
import sys

N_FEATURES = 39
N_ACTIONS = 5

class TargetAchiever:

    action_space = [2.5,1.25,0,-1.25,-2.5]

    def __init__(self) -> None:
        self.device = 'cuda'
        rospy.init_node('target_achiever')
        self.model = DQN(N_FEATURES,N_ACTIONS).to(self.device)
        self.model.load_state_dict(torch.load('/home/adam/catkin_ws/src/project/weights/thirdmodel.pth'))
        self.model.eval()
        self.world = World(0.4,0.4,5,5)
        print('Model loaded')

    def achieve_target(self,x,y):
        current_state = self.world.reset_and_spawn(x,y,delete_target= False)
        current_state = torch.tensor(current_state, dtype=torch.float32, device=self.device).unsqueeze(0)
        done = False
        while not done:
            action = self.model(current_state)
            current_state,done = self.world.eval_step(TargetAchiever.action_space[action.max(1).indices.view(1, 1)])
            current_state = torch.tensor(current_state, dtype=torch.float32, device=self.device).unsqueeze(0)
        self.world.reset_world()
        print('Target reached')

if __name__ == '__main__':
    
    a = TargetAchiever()
    while not rospy.is_shutdown():
        x,y = input('Give coords to achieve: ').split(' ')
        x,y = float(x),float(y)
        a.achieve_target(x,y)