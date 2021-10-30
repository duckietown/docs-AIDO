# ROS Template {#ros-template status=ready}

This section describes the basic procedure for making a submission with an agent
using the [Robot Operating System](http://www.ros.org/). 
It can be used as a starting point for any of the [`LF`](#challenge-LF), [`LFV`](#challenge-LFV), 
and [`LFI`](#challenge-LFI) challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Requires: That you have a basic understanding of [ROS](http://www.ros.org/).

Result:  You make a submission to all of the `LF*` challenges and can view their status and output.

</div>

<figure id="aido5-webinar-2-ros">
    <figcaption>ROS template</figcaption>
    <dtvideo src="vimeo:478452025"/>
</figure>

## Quickstart 

Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-ros):

    $ git clone git@github.com:duckietown/challenge-aido_LF-template-ros.git

Change into the directory:

    $ cd challenge-aido-LF-template-ros


Either make a submission with:

    $ dts challenges submit --challenge ![CHALLENGE_NAME]
    
where you can find a list of the open challenges [here](https://challenges.duckietown.org/v4/humans/challenges).


Or, run local evaluation with:

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]
    
    

### Verify the submission:

This will make a number of submissions (as described below). You can track the status of these submissions in the command line with:

    $ dts challenges follow --submission ![SUBMISSION_NUMBER]

or through your browser by navigating the webpage: `https://challenges.duckietown.org/v4/humans/submissions/![SUBMISSION_NUMBER]`

where `![SUBMISSION_NUMBER]` should be replaced with the number of the submission which is reported in the terminal output. 




## Anatomy of the submission

The submission consists of all of the basic files that required 
for a [basic submission](#minimal-template). 
Below we will highlight the specifics with respect to this template. 


There are also a few other **new** files and folders in this submission:

    launchers/
    submission_ws/

and additionally the `solution.py` is inside the `submission_ws` folder and `Dockerfile` have changed.
We will describe each of these in detail. 

Note: If you don't care about the details, or just want to get started, you can start by adding new ROS packages into the `submission_ws`.


### Dockerfile


The main update here is that we build your catkin workspace inside (the `submission_ws` folder) in the Dockerfile:

<pre trim="1" class="html">
<code trim="1" class="html">
RUN . /opt/ros/&#36;{ROS_DISTRO}/setup.sh &#38;&#38; \
    . &#36;{CATKIN_WS_DIR}/devel/setup.bash  &#38;&#38; \
    catkin build --workspace /code/submission_ws
</code>
</pre>

Also note that instead of just running `solution.py` when we enter the container, 
we now run a "launcher" (in the `launchers` folder) called `run_and_start.sh`. 
For details see [](#ros-template-launchers).



Also note that in this Dockerfile we are not copying the entire directory over, 
instead we are copying files individually (this is actually more efficient). 
So if you add new files that you are using that are outside of the `submission_ws` and `launchers` 
folders, you will have to add additional `COPY` commands. 



### `solution.py` {#ros-template-solution-py}

**You probably don't need to change this file.**

We instantiate a `ROSAgent()` (see [](#ros-template-rosagent-py))  and this becomes the object that handles interfacing with the ROS interface. 
This includes the publishing of imagery and encoder data to ROS:

```python
    def on_received_observations(self, data: DB20ObservationsWithTimestamp, context: Context):
        camera = data.camera
        odometry = data.odometry
        # context.info(f'received obs camera {camera.timestamp} odometry {odometry.timestamp}')

        if camera.timestamp != self.last_camera_timestamp or True:
            self.agent.publish_img(camera.jpg_data, camera.timestamp)
            self.agent.publish_info(camera.timestamp)
            self.last_camera_timestamp = camera.timestamp

        if odometry.timestamp != self.last_odometry_timestamp or True:
            self.agent.publish_odometry(
                odometry.resolution_rad, odometry.axis_left_rad, odometry.axis_right_rad, odometry.timestamp
            )
            self.last_odometry_timestamp = odometry.timestamp
```

Notice now that the protocol includes timestamps which are used to tag the data,  
and that a new camera image is not published if the timestamp does not change. 


### `rosagent.py` {#ros-template-rosagent-py}

**You probably don't need to change this file.**

`rosagent.py` sets up a class that can be used to interface with the rest of the ROS stack. It is for all intents and purposes a fully functional ROS node except that it isn't launched through ROS, it is instantiated in code. This class takes care of a few useful things,  such as getting the correct camera calibration files,  subscribing to control commands and sending them to your robot (real or simulated), as well as retreiving the sensor data from the robot and publishing it to ROS.

The main functions are:

 - `def publish_img(self, obs: bytes, timestamp: float):`, which takes the camera observation from the environment, and publishes it to the topic that you specify in the constructor of the `ROSAgent` 
 - `def publish_odometry(self, resolution_rad: float, left_rad: float, right_rad: float, timestamp: float):`, which take the encoder data from the robot, and publishes it to the topic specified in the constructor of the `ROSAgent`. 
 - `def _ik_action_cb(self, msg):`, listens on the inverse kinematics action topic, and assigns it to `self.action`. 



### `launchers/` {#ros-template-launchers}

The bash scripts in the `launchers` directory are there to help you get everything started when you run your container. In this template there is only `run_and_start.sh`:

<pre trim="1" class="html">
<code trim="1" class="html">
#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend

set -eux

dt-exec-BG roscore

dt-exec-BG roslaunch --wait random_action random_action_node.launch
dt-exec-FG roslaunch --wait agent agent_node.launch || true

copy-ros-logs

</code>
</pre>

You are free to modify this as you see fit, but a few things are important to consider. 

 1. The order that we `source` things matters. If we have a package with the same name in two workspaces, ROS will run whichever one got **sourced last**. 
 2. If you don't put things in the background (with `dt-exec-BG`) then if those commands don't end, subsequent commands will not get run.
 3. The `--wait` flag in the `roslaunch` command is recommended so that `roslaunch` will wait until the `roscore` has finished initializing. 


### `submission_ws/` {#ros-template-submission_ws}

This is a standard ROS catkin workspace. You can populate it with [ROS packages](http://wiki.ros.org/ROS/Tutorials/CreatingPackage). 
You will notice that the `random_action` package is already in the workspace. This can be used as a template for creating more packages. 
The main elements are launch files in the `launch` folder (you will see the `random_action_node.launch` which is launched by the `run_and_start.sh` launcher), 
the `src` folder which contains the ROS nodes, and the `include` folder which contains your python includes (you can also write nodes in C++ or other languages if you prefer). 

