# gst-autofocus
Autofocus gtreamer plugin for OPTIMOM module

# Version 1.2

# About

This plugin implements two autofocus algorithm.

## The naive algorithm

From the lower bound of the pda range the algorithm increase the pda value by a given step.
If the current sharpness calculated is the maximum found so far, it is saved.
When X consecutive images are blurrier the pda is set to 0 and  after waiting around 10ms go to the pda value corresponding to the maximum sharpness found.

This technic can be very precise, however it can be very slow if the target is close to the camera.

## The two pass algorithm
This algorithm has two phases.
In the first one the algorithm cover the entire pda range allowed with a given step.
At the same time the best sharpness values is saved with its corresponding pda value.
The new range for the second pass is on either side of the best sharpness
Then the second phase is like the naive algorithm but it uses the pda range found by the first phase.

This technic, if the step for both phases is well tuned, can achieve similar results to the naive algorithm, but much quicker.

# Dependencies

The following libraries are required for this plugin.
- v4l-utils
- libv4l-dev
- libgstreamer1.0-dev
- libgstreamer-plugins-base1.0-dev

### For compilation 
Note : gcc autotools and make are installed by default in most of linux distributions (should be installe on yocto images).

- gcc
- (autotools + make) or (meson + ninja) 

### For usage 

- gstreamer-1.0


Install them with: 

```
sudo apt install v4l-utils libv4l-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

# Compilation
First you must make sure that your device's clock is correctly setup.
Otherwise the compilation will fail.

In the AutofocusPlugin folder do:

	bash autogen.sh
	make

# Install

	sudo make install

To test if the plugin has been correctly install, do:

	export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0/
	gst-inspect-1.0 autofocus

If the plugin failed to install the following message will be displayed: ```No such element or plugin 'autofocus'```

# Uninstall

	sudo rm /usr/local/lib/gstreamer-1.0/libgstautofocus.*

# Usage

By default the plugin is installed in ```/usr/local/lib/gstreamer-1.0```. 
It is then required to tell gstreamer where to find it with the command:

	export GST_PLUGIN_PATH=/usr/local/lib/gstreamer-1.0/

The plugin can be used in any gstreamer pipeline by adding ```autofocus```, the name of the plugin.

To launch the autofocus, write ```a``` in the terminal then press enter.
To recalibrate the autofocus press ```c``` in the terminal then press enter

## Pipeline examples:
Simple test:

	gst-launch-1.0 v4l2src ! autofocus ! queue ! videoconvert ! queue ! xvimagesink sync=false

Manual control on defined ROI:

	gst-launch-1.0 v4l2src ! autofocus step_small=5 offset=2 continuous=false continuous_update_interval=5 x=860 y=440 width=200 height=100 ! queue ! videoconvert ! queue ! xvimagesink sync=false

Continuous autofocus:

	gst-launch-1.0 v4l2src ! autofocus step_small=5 offset=2 continuous=true continuous_update_interval=5 x=860 y=440 width=200 height=100 ! queue ! videoconvert ! queue ! xvimagesink sync=false

# Plugin parameters

- strategy: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 1
    - Default value: 1
    - Description: Set which algorithm is used to do the autofocus
        - 0 is the naive algorimth
        - 1 is the two pass algorimth

- x: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 1920
    - Default value: 0
    - Description: The top left X coordinates of the ROI

- y: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 1080
    - Default value: 0
    - Description: The top left Y coordinates of the ROI

- width: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 1920
    - Default value: 1920
    - Description: The width of the ROI

- height: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 1080
    - Default value: 1080
    - Description: The height of the ROI

- step_small: 
    - Type: int
    - Minimal value: 1
    - Maximal value: 700
    - Default value: 8
    - Description: The step of the PDA for the naive algorithm

- step_big: 
    - Type: int
    - Minimal value: 1
    - Maximal value: 700
    - Default value: 63
    - Description: The step of the PDA for the two pass algorithm

- pda_min: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 700
    - Default value: 200
    - Description: The minimal PDA value used for the autofocus algorithm

- pda_max: 
    - Type: int
    - Minimal value: 0
    - Maximal value: 750
    - Default value: 750
    - Description: The maximal PDA value used for the autofocus algorithm

- dec:
    - Type: int
    - Minimal value: 0
    - Maximal value: 20
    - Default value: 3
    - Description: The number of consecutive blurrier frames before stopping the autofocus

- offset:
    - Type: int
    - Minimal value: 0
    - Maximal value: 100
    - Default value: 3
    - Description: The frame offset between a pda command and the arrival of the corresponding frame in the plugin

- continuous:
    - Type: boolean
    - Default value: false
    - Description: Whether or not the autofocus should be restarted automaticly if it detect a loss of focus

- continuous_update_interval:
    - Type: int
    - Minimal value: 1
    - Maximal value: 120
    - Default value: 30
    - Description: How often should the sharpness be calculated

- continuous_threshold:
    - Type: float
    - Minimal value: 1.0
    - Maximal value: 100.0
    - Default value: 25.0
    - Description: The threshold to determine if the image is blurrier and if the autofocus should be relaunched

- continuous_timeout:
    - Type: int
    - Minimal value: 1
    - Maximal value: 100
    - Default value: 4
    - Description: How many times should the calculated sharpness be under the threshold before relaunching the autofocus algorithm
    - Note: The parameter has no effect if the parameter continuous is set to false

- listen:
    - Type: boolean
    - Default value: true
    - Description: Listen for user input in the terminal. 
                   This should be set to false for gstreamer application which handle user input on their own.

- debug_level:
    - Type: enum
    - Values:
        - none(0): Don't log any information about the autofocus
        - minimal(1): Only log the step, pda range and best focus found
        - full(2): Add on top of the minimal level information about each step of the algorithm
    - Default value: minimal
    - Description: Set level of debugging information

- pda_hold_cmd:
    - Type: int
    - Minimal value: 0
    - Maximal value: 1024
    - Default value: 0
    - Description: The number of frame between each command sent
