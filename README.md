# DeepQLearning TurtleBot
TurtleBot3 controller based on reinforement learning with posibility of creating remote model server on another device to relieve embedded device. Project performs point achieving with TurtleBot3 Gazebo simulations and ROS Noetic. Robot controller use a laser scan results and deep learning model, which is trained used emulated environment of Gazebo's simulation.

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install.

```bash
pip install -r requirements.txt
```
Change directory
```bash
cd catkin_ws/src
```
Clone TurtleBot3 gazebo package
```bash
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```
Clone project package
```bash
git clone https://github.com/jakadam2/DeepQLearningTurtleBot.git
```
Compile the packages
```bash
cd .. & catkin_make
```
## Usage

```bash
roslaunch DeepQLearningTurtleBot my_wordl.launch #starting a simulation
rosrun DeepQLearningTurtleBot model_server.py #on embedded device or remote
rosrun DeepQLearningTurtleBot get_target.py # point acchiever file

```

## License

[MIT](https://choosealicense.com/licenses/mit/)