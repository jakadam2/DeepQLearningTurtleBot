#! /usr/bin/env python

import rospy
from project.srv import Controller,ControllerResponse
from project.msg import Action
from model import DQN
import torch
import os
from pathlib import Path

N_FEATURES = 39
N_ACTIONS = 5

class ModelServer:

    action_space = [2.5,1.25,0,-1.25,-2.5]

    def __init__(self,model_name,n_observations,n_actions) -> None:
        rospy.init_node('model_server')
        self.model = DQN(n_observations,n_actions).to('cuda')
        model_path = os.path.join(Path(__file__).parent.parent,'weights',model_name + '.pth')
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()

    def model_server_response(self,req):
        state = req.state.state_array
        state = torch.tensor(state).to('cuda').unsqueeze(0)
        action = self.model(state)
        action = ModelServer.action_space[action.max(1).indices.view(1, 1)]
        return(ControllerResponse(Action(action)))

    def start(self):
        rospy.init_node('model_server')
        s = rospy.Service('Controller',Controller,self.model_server_response)
        print('SERVER STARTED')
        rospy.spin()


if __name__ == '__main__':
    server = ModelServer('model',N_FEATURES,N_ACTIONS)
    server.start()