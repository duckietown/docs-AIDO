# Duckietown Baseline {#ros-baseline status=ready}

This section describes the basic procedure for making a submission using the [Robot Operating System](http://www.ros.org/) and the [Duckietown software stack](https://github.com/duckietown/dt-core).

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template) and you 
understand how it works.

Requires: You already know something about ROS.

Result: You have a competitive submission.

</div>

<figure id="aido5-webinar-2-baseline">
    <figcaption>ROS template</figcaption>
    <dtvideo src="vimeo:478452025"/>
</figure>

## Quickstart

Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-duckietown)

    $ git clone git@github.com:duckietown/challenge-aido_LF-baseline-duckietown.git 

Change into the directory:

    $ cd challenge-aido_LF-baseline-duckietown

Test the submission, either locally with:

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]

or make an official submission when you are ready with 

    $ dts challenges submit --challenge ![CHALLENGE_NAME]

You can find the list of challenges [here][list-challenges]. Make sure that it is marked as "Open". 


[list-challenges]: https://challenges.duckietown.org/v4/humans/challenges


## Baseline Details {#duckietown-baseline-details}

The "Duckietown" baseline is based on the [ROS template](#ros-template).

### Dockerfile {#duckietown-baseline-dockerfile}

One important fact of the Dockerfile is that we use a "multi-stage build":

```
FROM ${DOCKER_REGISTRY}/duckietown/dt-car-interface:${BASE_TAG} AS dt-car-interface

FROM ${DOCKER_REGISTRY}/duckietown/challenge-aido_lf-template-ros:${BASE_TAG} AS template

FROM ${DOCKER_REGISTRY}/duckietown/dt-core:${BASE_TAG} AS base
```

This allows us to take some elements from each of the first two base images, and copy them into the `dt-core` image:

```
COPY --from=dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface ${CATKIN_WS_DIR}/src/dt-car-interface
COPY --from=template /data/config /data/config
COPY --from=template /code/rosagent.py .
```

As a result, we have the calibration files (from `/data/config`) as well as the `rosagent.py` from the `challenge-aido_lf-template-ros` and all the source files from the [`dt-car-interface`](https://github.com/duckietown/dt-car-interface) image.

We also get everything that is in the [`dt-core`](https://github.com/duckietown/dt-core) image. 

The remainder of the Dockerfile is very similar to the Dockerfile in the [ROS template](#ros-template).

### `solution.py` {#duckietown-baseline-solution-py}

There is no `solution.py` because it is inherited from the [ROS template](#ros-template). 
In the event that you wanted to, for example, change the launcher that was run in the final `CMD` line. 

### `launchers/` {#duckietown-baseline-launchers}

There is only one "launcher", and it deviates slightly from the one in the [ROS template](#ros-template):

```bash
#!/bin/bash

source /environment.sh

source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/solution/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend

set -eux

dt-exec-BG roscore

dt-exec-BG roslaunch --wait car_interface all.launch veh:="${VEHICLE_NAME}"
dt-exec-BG roslaunch --wait duckietown_demos lane_following.launch

sleep 5 # for some reason we still need this so that nodes can startup
dt-exec-BG roslaunch --wait duckietown_demos set_state.launch veh:="${VEHICLE_NAME}" state:="LANE_FOLLOWING"

rostopic list
# foreground
dt-exec-FG roslaunch --wait agent agent_node.launch || true
rostopic list
copy-ros-logs
```

Here we launch the [`lane_following.launch` launch file from the `duckietown_demos` package](https://github.com/duckietown/dt-core/blob/daffy/packages/duckietown_demos/launch/lane_following.launch). We don't go into the intricate details of everything that is run in this launch file here, but some of the more consequential nodes which are getting launched are the following:

- [line_detector_node](https://github.com/duckietown/dt-core/tree/daffy/packages/line_detector): Used to detect the lines in the image.
- [ground_projection_node](https://github.com/duckietown/dt-core/tree/daffy/packages/ground_projection): Used to project the lines onto the ground plane using the camera extrinsic calibration.
- [lane_filter_node](https://github.com/duckietown/dt-core/tree/daffy/packages/lane_filter): Used to take the ground projected line segments and estimate the Duckiebot's position and orientation in the lane.
- [lane_controller_node](https://github.com/duckietown/dt-core/tree/daffy/packages/lane_control): Used to take the estimate of the robot and generate a reference linear and angular velocities for the Duckiebot.

In the event that you wanted to, for example change the launcher that was run in the final `CMD` line. 


### `submission_ws/`

The `submission_ws` folder contains all the new ROS packages that you would like to include in your submission. It is currently empty, but there is a  reference package included in the [ROS template](#ros-template). 

Note: Importantly, your `submissions_ws` is sourced **after** the existing `catkin_ws` that is included in `dt-core`. As a result, if you include a node and package in your `submission_ws` *with the same name* as one in `dt-core`, the one in `submission_ws` will get executed. This is convenient because it means that, as long as you adhere to the same subscriptions and publications, you don't need to define any new launch file, `lane_following.launch` will automatically launch your newly written node. 


## Local Development Workflow {#duckietown-baseline-local-workflow}

For rapid local development, you can make use of the [`dts exercises` API](+opmanual_duckiebot#running-exercises), 
developed to build and test exercises and assignments in class settings. 

### Building your Code {#duckietown-baseline-building-code}

From inside the `challenge-aido_LF-baseline-duckietown` folder, you can start by building your code with:

    $ dts exercises build

This performs `catkin build` inside a docker container. If you go inside the `submission_ws` folder you will notice that there are more folders that weren't there before. These are build artifacts that persist from the building procedure because of mounting. 
    
### Running in Simulation {#duckietown-baseline-running-simulation}

You can run your current solution in the gym simulator with:
 
     $ dts exercises test --sim
     
Then you can look at what's happening by looking through the browser at [http://localhost:8087](http://localhost:8087). This will open a noVNC desktop. In it,
open up the `rqt_image_view`, resize it, and choose `/agent/camera_node/image/compressed` in the dropdown. You should see the image from the robot in the simulator. 
 
 You might want to launch a virtual joystick by opening a terminal and doing:
 
    $ dt-launcher-joystick
     
By default the Duckiebot is in joystick control mode, so you can freely drive it around. You can also set it to `LANE FOLLOWING` mode by pushing the `a` button when you have the virtual joystick active. If you do so you will see the robot move forward slowly and never turn. 

At the same time, you can see a birds eye overview of the Duckiebot on the track though the browser at [http://localhost:8090](http://localhost:8090).

### Testing Your Algorithm on the Robot {#duckietown-baseline-testing-robot}

If you are using a Linux laptop, you have two options, local (i.e., on your laptop) and remote (i.e., on the Duckiebot). To run "locally"

     $ dts exercises test --duckiebot_name ![ROBOT_NAME] --local

To run on the Duckiebot:

    $ dts exercises test --duckiebot_name ![ROBOT_NAME]

In both cases you should still be able to look at things through noVNC by pointing your browser to  [http://localhost:8087](http://localhost:8087) . If you are running on Linux, you can load up the virtual joystick and start lane following as above. 

Warning: If you are Mac user unfortunately you should not use the `--local` flag


#### Starting Lane Following on Mac
TODO: should be retested

Since we can't publish from Mac and have it be received by ROS, we have to do something slightly different. In a new terminal on your Mac do:

    $ docker -H ![ROBOT_NAME].local exec agent launchers/start_lane_following.sh
    
This will run the `start_lane_following.sh` bash script inside the agent container which initiates `LANE_FOLLOWING` mode. 

Similarly, you can stop your Duckiebot from lane following by doing:

    $ docker -H ![ROBOT_NAME].local exec agent launchers/stop_lane_following.sh

You could also do an equivalent thing through the Portainer interface in the dashboard by opening a new terminal in your agent container and running the corresponding launcher. 


### How to Improve your Submission {#duckietown-baseline-improve}


A good way to get started could be to copy one of the packages defined in the [Duckietown dt-core repo](https://github.com/duckietown/dt-core) or the [Duckietown dt-car-interface repo](https://github.com/duckietown/dt-car-interface) into the `submission_ws` folder and modify it. Note that your modified package will automatically get run because of the order of the sourcing of the catkin workspaces in the `run_and_start.sh` launch file.

If you would like to add a new package and node that includes a functionality not already run by `lane_following.launch` or you would like to change the connectivity of interfaces of these nodes, then you will also need:

 - to write your own launch file that launches your node and also all of the other nodes from the base images that you would still like to use. 
 - to modify the launch file `run_and_start.sh` so that it launches your newly created launchfile. You could equally define a new launchfile, but then make sure that it gets executed in the last line of your `Dockerfile`. 
 

### Other Possibly Useful Utilities {#duckietown-baseline-other-utilities}
  
All of the normal ROS debugging utilities are available to you through the noVNC desktop. For example, 
You might also explore the other outputs that you can look at in `rqt_image_view`. 

Also useful are some debugging outputs that are published and visualized in `RViz`. 
You can open `RViz` through the terminal in the noVNC desktop by typing:

    $ rviz
    
In the window that opens click "Add" the switch to the topic tab, then find the `segment_markers`, and you should see the projected segments appear. Do the same for the `pose_markers`. 

Another tool that may be useful is `rqt_plot` which also can be opened through the terminal in noVNC. This opens a window where you can add "Topics" in the text box at the top left and then you will see the data get plotted live. 

All of this data can be viewed as data through the command line also. Take a look at all of the `rostopic` command line utilities. 

