#! /usr/bin/env python
from world import World
from time import sleep
import rospy
rospy.init_node('test')
w = World(0,0,1,1)
w.spawn_target()
print(w.target_position())
sleep(2)
w.delete_model('target')
sleep(2)
print(len(w.get_laser_scan()))
w.teleport_robot('turtlebot3_burger',1,1)
w.start_robot()
sleep(3)
print(w.robot_position())
w.stop_robot()
sleep(1)
w.reset_world()



'''
TODO:
-make target and robot position as property
-repair launch to stop it throwing errors (but despite of errors it work)
'''