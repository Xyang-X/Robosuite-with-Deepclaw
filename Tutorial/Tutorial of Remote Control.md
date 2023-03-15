# Tutorial of Remote Control

In this section, you will learn to implement remote control  in robots in robosuite, especially with Deepclaw.  The relative code will be used are available in [github](https://github.com/Xyang-X/Robosuite-with-Deepclaw). 

You can clone the repository into your workspace.

```sh
$ git clone https://github.com/Xyang-X/Robosuite-with-Deepclaw.git
```



## Remote control with keyboard

Robosuite provided official IO for keyboard and 3D-mouse to transform human demonstration into robots. You can try to run it. More information from [wiki page](https://robosuite.ai/docs/algorithms/demonstrations.html) 

1. Run the script by terminal in your workspace

```sh
$ cd Robosuite-with-Deepclaw/scripts
$ python collect_human_demonstrations.py --directory 'demonstrations'
```

The data of trajectory will be record as `hdf5` file in a directory named `demonstrations`.

![Aaron Swarts](https://github.com/Xyang-X/Robosuite-with-Deepclaw/blob/main/Tutorial/img/remote_keyboard.png?raw=true)

Now you can use keyboard to control the robot. Try to grip the cube on table.(in the demo, only have the cube lifted up, a demonstration would be record)

2. replay your demonstration

   ```sh
   $ python playback_demonstrations_from_hdf5.py --folder 'demonstration'
   ```



## Remote control with Deepclaw

By transforming the variation of pose detected by Deepclaw into the `OSC_POSE` controller in robosuite, we can implement remote control in a more flexible way.

1. Create environment instance 

   ```python
   import sys
   import numpy as np
   import robosuite as suite
   from Robosuite-with-Deepclaw.remoteDetc import RemoteDetector
   import matplotlib.pyplot as plt
   from robosuite import load_controller_config
   from robosuite.utils.input_utils import input2action
   from robosuite.wrappers import DataCollectionWrapper, VisualizationWrapper
   from robosuite.scripts.collect_human_demonstrations import gather_demonstrations_as_hdf5
   import os
   import json
   import datetime
   
   def load_sim(tmp_directory):
       '''creat a simulating environment in robosuite'''
       controllerconfig = suite.load_controller_config(default_controller="OSC_POSE")
       config = {
           "env_name": "Stack",
           "robots": "Panda",
           "controller_configs": controllerconfig,
       }
       env = suite.make(
           **config,
           has_renderer=True,
           has_offscreen_renderer=False,
           use_camera_obs=False,
           render_camera="frontview",
           control_freq=20,
       )
       
       env = VisualizationWrapper(env)
       env = DataCollectionWrapper(env, tmp_directory)
       return env, config
   ```

2. Initialize Deepclaw

   ```python
   detector = RemoteDetector()
   ```

   `RemoteDetector` is a class defined in `remoteDetc.py` to read data from Deepclaw. It contains 3 optional arguments: `config_path`: the path of the file record camera parameter you collected last lab; 

   `marker_len`: the length of Aruco markers,  default value : 0.015(in meter); 

   `camera_id`: the index of camera, default value: 0(try other numbers if failed to open camera of Deepclaw).

   

   It provide 2 functions to 

   `single_marker_control`: outputs the pose data of one assigned marker.

   `single _gripper_control`: output the pose of center of tongs and the state of  gripper(open of closed)

3. start to run

   ```python
   while True:
           now = datetime.datetime.now()
           tmp_directory = os.path.join(os.getcwd(),
                                        "tmp/{}_{}{}_{}{}{}".format(now.year, now.month, now.day, now.hour, now.minute,now.second)) # dictory to record raw data
           new_dir = os.path.join(os.path.join(os.getcwd(), "demonstrations"),
                                  "{}_{}{}_{}{}{}".format(now.year, now.month, now.day, now.hour, now.minute, now.second)) # dictory to record trajectory in hdf5 file
           os.makedirs(new_dir)
           
           env0, config = load_sim(tmp_directory) # create new environment everytime
           env0.reset()
           env_info = json.dumps(config)
           while True: # record a period of demonstration in every subloop
               action = detector.single_gripper_control() # read command from deepclaw
               
               env0.successful = True
               env0.step(action) # transfrom command into robot controller
               env0.render() # render change into environment
               if detector.record or detector.quit == True: # end subloop
                   detector.record = False
                   break
           env0.close() # close simulation environment and record raw data
           gather_demonstrations_as_hdf5(tmp_directory, new_dir, env_info) # transform raw data into hdf5
           if detector.quit: # end main loop
               break
   ```

   In this demo, you can use keyboard to input command:

   `s` : save trajectory and start new record;

   `q` : quit program.

![Aaron Swats](https://github.com/Xyang-X/Robosuite-with-Deepclaw/blob/main/Tutorial/img/remote_sim_view.png?raw=true) 

![Aaron Swarts](https://github.com/Xyang-X/Robosuite-with-Deepclaw/blob/main/Tutorial/img/remote_cameraview.png?raw=true)

