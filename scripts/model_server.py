#! /usr/bin/env python

import rospy
from project.srv import Controller,ControllerResponse
from project.msg import Action
from model import DQN
import torch
import sys
import os
from pathlib import Path

class ModelServer:

    def __init__(self,model_name,n_observations,n_actions) -> None:
        rospy.init_node('model_server')
        self.model = DQN(n_observations,n_actions).to('cuda')
        model_path = os.path.join(Path(__file__).parent.parent,'weights',model_name + '.pth')
        self.model.load_state_dict(torch.load(model_path))
        self.model.eval()
        pass

    def model_server_response(self,req):
        state = req.state.state_array
        state = torch.tensor(state).to('cuda').unsqueeze(0)
        print(f'GOT STATE {state}')
        return(ControllerResponse(Action(0)))
        pass

    def start(self):
        rospy.init_node('model_server')
        s = rospy.Service('Controller',Controller,self.model_server_response)
        print('SERVER STARTED')
        rospy.spin()