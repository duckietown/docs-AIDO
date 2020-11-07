# Classical Duckietown Baseline (ROS) {#ros-baseline status=ready}

This section describes the basic procedure for making a submission using the [Robot Operating System](http://www.ros.org/) and the  [Duckietown software stack](https://github.com/duckietown/dt-core) .

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template) and you understand how it works.

Requires: You already know something about ROS.

Result: You have a competitive submission

</div>

## Quickstart

### Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-duckietown)

    $ git clone git://github.com/duckietown/challenge-aido_LF-baseline-duckietown.git 

### Change into the  directory:

    $ cd challenge-aido_LF-baseline-duckietown
    
### Test the submission

Either locally with 

    $ dts challenges evaluate --challenge ![CHALLENGE_NAME]
    
Or make an official submission when you are ready with 

    $ dts challenges submit ![CHALLENGE_NAME]
    
You can find the list of challenges [here](https://challenges.duckietown.org/v4/humans/challenges). Make sure that it is marked as "Open". 


## Local Development Workflow

More details to come. 

### How to Improve your Submission {#ros-workflow}


You will notice one main difference as compared to the [ROS template](#ros-template) is that the launcher launches the `lane_following.launch` file which runs the basic lane following stack. 
These are nodes that are defined in the [Duckietown dt-core repo](https://github.com/duckietown/dt-core) and the [Duckietown dt-car-interface repo]](https://github.com/duckietown/dt-car-interface), which is included  since the base images in the Dockerfile are `duckietown/dt-car-interface` and `duckietown/dt-core` . 

The nodes which are getting launched include (among others) the following:

 - [line_detector_node](https://github.com/duckietown/dt-core/tree/daffy/packages/line_detector): Used to detect the lines in the image.
 - [ground_projection_node](https://github.com/duckietown/dt-core/tree/daffy/packages/ground_projection): Used to project the lines onto the ground plane using the camera extrinsic calibration.
 - [lane_filter_node](https://github.com/duckietown/dt-core/tree/daffy/packages/lane_filter): Used to take the ground projected line segments and estimate the Duckiebot's position and orientation in the lane
 - [lane_controller_node](https://github.com/duckietown/dt-core/tree/daffy/packages/lane_control): Used to take the estimate of the robot and generate a reference linear and angular velocities for the Duckiebot

A good way to get started could be to copy one of these packages into the `submission_ws` folder and modify it. Note that your modified package will automatically get run because of the order of the sourcing of the catkin workspaces in the `run_and_start.sh` launch file. 


