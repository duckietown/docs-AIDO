# Classical Duckietown Baseline (ROS) {#ros-baseline status=ready}

This section describes the basic procedure for making a submission using the [Robot Operating System](http://www.ros.org/) and the  [Duckietown software stack](https://github.com/duckietown/Software) .

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template) and you understand how it works.

Requires: You already know something about ROS.

Result: You could win the AI-DO!

</div>

## Quickstart

### Clone this [repo](https://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown)

    $ git clone git://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown.git

### Change into the directory you cloned

    $ cd challenge-aido1_LF1-baseline-duckietown
    

### Evaluate your submission

Either locally with 

    $ dts challenges evaluate
    
Or make an official submission when you are ready with 

    $ dts challenges submit
    

## Workflows

Here we will present 3 distinct workflows for improving your submission:

1. Using `dts challenges evaluate` [](#ros-workflow)
2. Running everything, including the simulator, locally and building your own new nodes [](#ros-running-locally)
3. Running everything locally and directly modifying the software repo locally [](#ros-software-locally)

One efficient option might be to start with 3 - test your code really fast inside the software repo, then move to number 2 - pull your new code (packages) out of the software repo and run them independently, then finally move to 1 - move your node over to the baseline and see what score you get. 


## How to Improve your Submission {#ros-workflow}


You will notice one main difference as compared to the [ROS template](#ros-template) is that the launch file argument in the `solution.py` code now points to `lf_slim.launch`, which launches a slimmed-down version of the [Duckietown Lane Following code](https://github.com/duckietown/Software/). The action and image topics are both adjusted to match the inputs and outputs of the original stack.

Inside of the Dockerfile, you will also see how to build and maintain your own `catkin_ws`. While in this example the workspace is not used, you will surely need it to build your own code.

### lf_slim.launch

Compared to `template.launch` in [](#ros-template), the launch file here actually runs some nodes. These are nodes that are defined in the [Duckietown Software repo](https://github.com/duckietown/Software/). 

The nodes which are getting launched are the following:

 - [line_detector_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/line_detector): Used to detect the lines in the image.
 - [ground_projection_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/ground_projection): Used to project the lines onto the ground plane using the camera extrinsic calibration.
 - [lane_filter_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/lane_filter): Used to take the ground projected line segments and estimate the Duckiebot's position and orientation in the lane
 - [lane_controller_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/lane_control): Used to take the estimate of the robot and generate a reference linear and angular velocities for the Duckiebot
 - [inverse_kinematics_node](https://github.com/duckietown/Software/blob/master18/catkin_ws/src/05-teleop/dagu_car/src/forward_kinematics_node.py): Take the refernece linear and angular velocities and generate left and right wheel velocities which are published to the `rosagent`.

Note: You don't see the software repo in the baseline but it will be there when your docker container is built since the image inherits from the `rpi-duckiebot-base` image which has the code.

A good way to get started could be make your own version of the nodes that are being launched in `tf_slim.launch`.

Do this by building your own package (with a different name) in your `src/catkin_ws`. If you like you can copy the source files from the `Software` repo as a starting point. 

You will need to define your own launch file and parameters in your package and then make sure that they make sure they are being launched and loaded properly in `tf_slim.launch`. 


## Running the Entire Thing Locally for Debugging {#ros-running-locally status=ready}

You can evaluate locally using `dts challenges evaluate`, but you may find this restricting, especially when you want to use ROS debugging tools like the `rostopic` or `roslogging` interfaces. For convenience, we've also provided a standalone version of this code (which will not give any rewards or AIDO task evaluation, but provides a way to easily visualize the output and behavior of your code) which can be found [here](https://github.com/duckietown/sim-duckiebot-lanefollowing-demo). This code will not be able to run on the Duckiebot, since it extends the `ros:kinetic` image, which by default, will not build the ARM version.

Clone the [standalone repo](https://github.com/duckietown/sim-duckiebot-lanefollowing-demo):

    $ git clone git@github.com:duckietown/sim-duckiebot-lanefollowing-demo.git


The interface is mainly the same, except now, the `rosagent.py` file itself controls the simulation. Again, you will mainly want to focus on `rosagent.py`, and you will again be able to see the `Dockerfile` for how to build and maintain your own `catkin_ws`.


To run this, you will need to have `docker-compose` installed on your local machine, as unlike the AIDO submissions, this will emulate both the server and agent all on your local machine. Follow instructions [here](https://docs.docker.com/compose/install/) to install.


## Usage

To launch the lane following demo, run the following command:
    
    $ docker network create gym-duckietown-net
    $ docker-compose -f docker-compose-lf.yml pull
    $ docker-compose -f docker-compose-lf.yml build
    $ docker-compose -f docker-compose-lf.yml up
    
The first two commands don't need to be run every time, so after pulling, you may just want to run the `up` command.

You will then start to see output from the Lane Following code, which can be found [here](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control)

You can terminate the run at any time by pressing <kbd>CTRL</kbd>+<kbd>c</kbd>.

## Write your own agent

To write your own ROS agent, first fork this repository. Since we are going to be running a few containers, the best way to run is the `docker-compose` command found above.

Inside of the `docker-compose-lf.yml` file, you'll find that for purposes of this demo, we are using the `HOSTNAME=default`; the `HOSTNAME` can be thought of as the vehicle name. This is to help mitigate the discrepencies between the real robot and simulator when finding things like configuration files when using the old Duckietown stack.

### Making Edits

With `docker-compose`, your Dockerfiles will not rebuilt unless you tell them. There are two ways of going about this:

1. To rebuild everything, run `docker-compose -f docker-compose-lf.yml build [--no-cache]` before running `docker-compose -f docker-compose-lf.yml up`
2. (Preferred) Rebuild the container you've changed with `docker build -t "{your-containers-tag}" -f {corresponding-Dockerfile}` . and then `docker-compose -f docker-compose-lf.yml up`.

## Using your own ROS Nodes / Custom `catkin_ws`

Most likely, you'll want to work off of some of the standalone Duckietown code, but change a node or two. We will look at two examples:

(A) Adding to the pipeline
(B) "Cutting" the pipeline and inserting your node inside. 

Currently, you'll see that we have a `rosrun` command all the way at the bottom of the `docker-compose` file, which is where you'd put your `roslaunch` or `rosrun` command. We've provided an example node for you, that builds from `DockerfileCatkin`

To add your node to the pipeline, we give some simple example code. The files we're concerned with are:

`DockerfileCatkin`: To add your nodes to the `custom_ws`, follow the commented out instructions, paying close attention to which lines you should and should not be removing. We use something called [overlayed ROS workspaces](http://wiki.ros.org/catkin/Tutorials/workspace_overlaying), to make sure that your code (which most likely depends on the Duckietown ROS stack in some way) can find all of its dependencies.

`dt_dependent_node`: A simple, toy example of how to build a node that has a dependency with the current stack. You can use this as a model to build and add your own ROS nodes, making sure to edit the `CMakeLists.txt` (inside of your node, for dependencies + building things like msgs and services) and the Dockerfile to ensure your files and folders get copied into the `cudtom_ws/src` directory before you build with catkin_make.

`custom_line_detector`: A *copied* node from `10-lane-control` inside of the `Software` repo, we also provide this as an example of how to copy, edit, build, and launch a node. This serves as an example, and is commented out in `lf_slim.launch`. Remember, when copying a node, you either need to make sure that (A) that copied node isn't running with the same name elsewhere (just copy it out in the launch file) and that (B) you remap the topics properly.

`lf_slim.launch`: A launch file that launches the whole lane following stack, but at the bottom has the code to launch our simple test node. It launches nodes just the way you normally might in ROS, and because our workspaces are overlayed, will be able to find code or nodes in both your new workspace, as well as the old one.

To (B) Cut the pipeline, and insert your node in, you'll want to make use of [`remap` in the launch files](http://wiki.ros.org/Remapping%20Arguments). Simply take the topics you need from the last node from the existing pipeline, and remap them to what you're node takes in (usually, the node name will come first, to help ambiguities between nodes). Then, add your node(s), chaining them together with the remapping, and finally, remap your last nodes output to the topic you're interested in using - whether it be another node in the existing pipeline, or just the WheelCmd message that the rosagent.py is looking for to step in the environment.


## Cloning the Software Repo Locally {#ros-software-locally}

You may still find the above a little bit cumbersome because if you just wanted to change a single parameter in the existing code and see the result, you will have to copy the entire node over and modify all the launch files etc. Then you might discover that the parameter you modified wasn't the right one and this was a huge waste of time. Instead, we might like to be able to just modify things locally in the software repo and see the results in the simulator.

To do so clone the same directory as above:

    $ git clone git@github.com:duckietown/sim-duckiebot-lanefollowing-demo.git

Then move into the directory

    $ cd sim-duckiebot-lanefollowing-demo
    
Then move into the branch `local-software-repo`

    $ git checkout local-software-repo

Then clone the software repo here:

    $ git clone git@github.com:duckietown/Software.git

Then follow the same workflow as above:

    $ docker network create gym-duckietown-net 
    $ docker-compose -f docker-compose-lf.yml pull
    $ docker-compose -f docker-compose-lf.yml build
    $ docker-compose -f docker-compose-lf.yml up
    
You can go ahead and make changes in the `Software` directory and these changes will propagate into your container. This is done by mounting the `Software` directory as a volume which effectively overwrites the existing software repo in the container. 

Note: If you have local nodes as described in [](#ros-running-locally), they will not be available now since the whole software directory is being overwritten.

Note: If you have only modified python script files that don't need to even be rebuilt, then you can replace `docker-compose -f docker-compose-lf.yml up` with

    $ docker-compose -f docker-compose-lf-no-build.yml up
    
    

## Debugging and Monitoring {#ros-debugging}

With ROS, everything of interest is passed through the ROS Messaging system. There are two ways to monitor your progress:

### ROSBags

Inside of the `docker-compose-lf.yml` file, you will find a node called `rosmonitor`, which listens on a particular topic and records a bagfile to a mounted drive. This is so your container and host machine can read and write to the same disk location. Once you've recorded the bag file, you can play its contents back on your host machine with the following steps:
1. You can start a `roscore` in one terminal, and in another terminal, you will want to type in `rosbag play {PATH TO BAGFILE}`. Some nice additional command line options are `--loop` or `--rate {#}`.
2. Now, your host machine is in the same state as the Docker image when the bag was recorded. This means you can visualize the messages with things like `rqt` or `image_view`.

### Via Docker in Real Time

If you'd like to monitor the progress of your system realtime via the ROS messaging system, you can also connect to the same network from another Docker container, and monitor or record ROSBags in real time. To do this, you will need to run a command:

`docker run --entrypoint=qemu3-arm-static --network=gym-duckietown-net -it duckietown/rpi-gui-tools:master18 /bin/bash`

If you want to run ROS plugins such as `rqt*` or other graphical tools, you will need to [enable X11 forwarding](http://wiki.ros.org/docker/Tutorials/GUI#Using_X_server). To do so, you will need to first run `xhost +` on your host machine to allow incoming connections, then add the following flags to the above `docker run` command: `--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`.

Which will give you a bash shell inside of a Duckietown-compatible Docker container (we can't use a normal ROS Kinetic container due to the fact that we need the Duckietown-specific messages to be built).

Inside of the shell, you will need to `export ROS_MASTER_URI=http://lanefollow:11311`, which will point to the ROS Master currently running in the `lanefollow` container.

### Troubleshooting

To check the available networks, run `docker network ls`. Occasionally Docker will create a second network, `sim-duckiebot-lanefollowing-demo_gym-duckietown-net` if the default one has already been created. If you want to override this behavior run `docker-compose up --force-recreate` to start everything from scratch.

