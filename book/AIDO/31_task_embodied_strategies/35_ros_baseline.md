# Classical Duckietown Baseline (ROS) {#ros-baseline status=ready}

This section describes the basic procedure for making a submission using the [Robot Operating System](http://www.ros.org/) and the  [Duckietown software stack](https://github.com/duckietown/Software) .

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template).

Result: You win the AI-DO!

</div>

## General Workflow {#ros-workflow}

First make sure you have followed the instructions in [](#ros-template).

If you enter into the directory `challenges-aido1-LF1-template-ros` that you clone in [](#ros-template), you will find a few important files:
    
    solution.py # Where your solution will live, and where the roslaunch command is run
    template.launch # The launchfile to start nodes and a ROSMaster
    rosagent.py # An interface to bridge your nodes and the submission interface


The workflow is as follows. The evalutor will launch `solution.py`. This file includes calls to the `roslaunch` API, which points to and launches `template.launch`. In more [involved][#lanefollowing-ros-baseline] implementations, this launch file will also launch other nodes, but in this minimal example, its only purpose is to launch the ROSCore. After this call occurs (`roslaunch` occurs asynchronously), the code instantiates a `ROSAgent`, which will serve as the interface to communicate with the ROS Nodes you launch. 

The three main functions are:
- `_publish_img(observation)`, which takes the observation from the environment, and publishes it to the topic that you specify in the constructor of the `ROSAgent` (You will want to replace the placeholder with the topic name that your node is expecting the image to come through on)
- `_action_cb(msg)`, which listens to the action topic you specify and pulls the `Twist2DStamped` message 
- `_ik_action_cb(msg)`, listens on the inverse kinematic action topic, and assigns it to `self.action`. Because we are using wheel velocities as the action, but the output of many Duckietown nodes is a `Twist2DStamped` message, we need a way to convert this back into the actions we are interested in.

The loop in `solution.py` pulls the action from `self.action`. This same loop also executes that action in the environment, and will continue to execute that action until you tell it otherwise. 


You will notice that there is a publisher which publishes random actions to the action topic. You will want to change this when you make your submission.

Back inside of the `solution.py` code, you will see calls to `rospy.Rate()` and `r.sleep` - these control the rate at which the stepping loop runs, and you may need to change this parameter to suit latencies in your own code.


## Lane Following (from the Duckietown Stack) ROS Baseline {#lanefollowing-ros-baseline status=ready}

We also provide a more-involved, working example of Lane Following, which extends the image described above: `duckietown/challenge-aido1_LF1-template-ros:v3`. Since it extends the random template, you will again be able to run this code on both the simulator and Duckiebot.


Clone the [baseline repo](https://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown):

    $ git clone git@github.com:duckietown/challenge-aido1_LF1-baseline-duckietown.git


You will notice that the launchfile argument in the `solution.py` code now points to `lf_slim.launch`, which launches a slimmed-down version of the [Duckietown Lane Following code](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control). The action and image topics are both adjusted to match the inputs and outputs of the original stack. 


Inside of the Dockerfile, you will also see how to build and maintain your own `catkin_ws`. While in this example the workspace is not used, you will surely need it to build your own code.


## Running Locally {#ros-running-locally status=ready}

In the debugging phase, you may notice that the submission interface for the AIDO Lane Following Challenge may be slightly restricting, especially when you want to use ROS debugging tools like the `rostopic` or `roslogging` interfaces. For convenience, we've also provided a standalone version of this code (which will not give any rewards or AIDO task evaluation, but provides a way to easily visualize the output and behavior of your code) which can be found [here](https://github.com/duckietown/sim-duckiebot-lanefollowing-demo). This code will not be able to run on the Duckiebot, since it extends the `ros:kinetic` image, which by default, will not build the ARM version.

Clone the [standalone repo](https://github.com/duckietown/sim-duckiebot-lanefollowing-demo):

    $ git clone git@github.com:duckietown/sim-duckiebot-lanefollowing-demo.git


The interface is mainly the same, except now, the `rosagent.py` file itself controls the simulation. Again, you will mainly want to focus on `rosagent.py`, and you will again be able to see the `Dockerfile` for how to build and maintain your own `catkin_ws`.


The instructions on the repository walk you through the steps to get started. To run this, you will need to have `docker-compose` installed on your local machine, as unlike the AIDO submissions, this will emulate both the server and agent all on your local machines.
