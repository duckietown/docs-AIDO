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


The instructions on the repository walk you through the steps to get started. To run this, you will need to have `docker-compose` installed on your local machine, as unlike the AIDO submissions, this will emulate both the server and agent all on your local machine.
