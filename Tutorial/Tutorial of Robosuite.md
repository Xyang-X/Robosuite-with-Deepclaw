# Tutorial of Robosuite

Robosuite is a simulation framework powered by the MuJoCo physics engine for robot learning. You can click [here](https://robosuite.ai/) for more information. In this section we will start from installation of robosuite. After that, you will learn how to load and control a robot in  robosuite.
## Installation

### Install with `pip`

```undefined
$ pip install mujoco
$ pip install robosuite
```

Test installation with

```undefined
$ python -m robosuite.demos.demo_random_action
```

### Install from source

1. Clone the robosuite repository

```undefined
$ git clone https://github.com/StanfordVL/robosuite.git
$ cd robosuite
```

2. Install the base requirements with

```undefined
$ pip install -r requirements.txt
```

3. Test installation with

```undefined
$ python -m robosuite.demos.demo_random_action
```

## Quickstart

### Running Standardized Environments

```Python
import numpy as np
import robosuite as suite
# create environment instance
env = suite.make(
env_name="Lift", # try with other tasks like "Stack" and "Door"
robots="Panda", # try with other robots like "Sawyer" and "Jaco"
has_renderer=True,
has_offscreen_renderer=False,
use_camera_obs=False,
)
# reset the environment
env.reset()
for i in range(1000):
action = np.random.randn(env.robots[0].dof) # sample random action
obs, reward, done, info = env.step(action) # take action in the environment
env.render() # render on display
```

## Building Your Own Environment

1. Load environment

```Python
import robosuite as suite
env = suite.make(
        env_name="Stack",  # try with other tasks like "Stack" and "Door"
        robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
        has_renderer=True, 
        has_offscreen_renderer=False,
        use_camera_obs=False,
        render_camera="frontview", # try with other views like "sideview" and "birdview
        controller_configs=controllerconfig # Load preset controller
    )
    env.reset()
```

1. Render environment

```Python
env.render()
```

If succeed, you would see:

![Aaron Swartz](https://github.com/Xyang-X/Robosuite-with-Deepclaw/blob/main/img/robot_demo.png?raw=true)

 	 



## Control your robot

### 1. Without controller(directly controlling by joints)

```Python
import robosuite as suite
env = suite.make(
        env_name="Stack",  # try with other tasks like "Stack" and "Door"
        robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
        has_renderer=True, 
        has_offscreen_renderer=False,
        use_camera_obs=False,
	control_freq=20, # set control frequency as 20Hz
        render_camera="frontview", # try with other views like "sideview" and "birdview
    )
env.reset()

action=[] # arry with 7 variable to control the currrent of joint 0～6 & gripper
env.step(action)
env.render()
```

### 2. Use controller to control  

Here we use `OSC_POSE` controller(operational space control),  other controllers like `OSC_POSITION`, `JOINT_POSITION`, `JOINT_VELOCITY`, and `JOINT_TORQUE`  are also provided in robosuite. 

```Python
import robosuite as suite
controllerconfig = suite.load_controller_config(default_controller="OSC_POSE")
# choose the controller

env = suite.make(
    env_name="Door",  # try with other tasks like "Stack" and "Door"
    robots="Panda",  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    render_camera="sideview",
    control_freq=20, # set control frequency as 20Hz
    controller_configs=controllerconfig # load the preset controller into environment
)
```

The input of `OSC_POSE` controller is` [x,y,z,rx,ry,rz,gripper]` . It represent the variation in 6-D pose and state of gripper(positive input is close and negative input is open). The units are meter and radian.

```python
action=[] # try to input an order you want
for i in range(1000)
	env.step(action)
    env.render()
```



