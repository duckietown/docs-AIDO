# Duckietown Baseline {#ros-baseline status=ready}

This section describes the current `ente` ROS lane-following baseline built on top of the [ROS template](#ros-template) and the [Duckietown software stack](https://github.com/duckietown/dt-core).

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template) and you 
understand how it works.

Requires: You already know something about ROS.

Result: You can build, evaluate, and customize the stock ROS lane-following baseline for `aido-LF-sim-validation`.

</div>

## Quickstart

Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-duckietown)

    $ git clone git@github.com:duckietown/challenge-aido_LF-baseline-duckietown.git 

Change into the directory:

    $ cd challenge-aido_LF-baseline-duckietown

The repository already targets `aido-LF-sim-validation` in `submission.yaml`.

Test the submission, either locally with:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

or make an official submission when you are ready with:

    $ dts challenges submit

You can find the list of challenges [here][list-challenges]. Make sure that it is marked as "Open". 


[list-challenges]: https://staging-challenges.duckietown.com/humans/challenges


## Baseline Details {#duckietown-baseline-details}

The "Duckietown" baseline is based on the [ROS template](#ros-template).

### Dockerfile {#duckietown-baseline-dockerfile}

The current Dockerfile starts from the `ente` ROS template image and adds only the pieces that are baseline-specific:

    FROM duckietown/challenge-aido_lf-template-ros:BASE_TAG

On top of the template, the baseline Dockerfile:

- Installs the baseline Python dependencies.
- Refreshes the Duckietown Python runtime dependencies from `dependencies.txt`.
- Installs the runtime camera calibration defaults into `/data/config/calibrations`.
- Overlays the local `solution/.` tree into the template project's `packages/agent` directory.
- Rebuilds the catkin workspace.
- Copies the custom launchers into `/code/launchers/`.

The result is a small baseline-specific overlay on top of the template rather than a separate full submission stack.

### `launchers/` {#duckietown-baseline-launchers}

The key baseline behavior lives in `launchers/run_and_start.sh`. Compared to the template launcher, it adds the stock Duckietown lane-following stack and waits for the system to be ready before forcing `LANE_FOLLOWING` mode. The full script sources the built workspaces, waits for camera and FSM readiness, and then follows this execution order:

    #!/bin/bash
    source /environment.sh
    source /opt/ros/noetic/setup.bash
    set -euxo pipefail
    dt-exec-BG roscore
    dt-exec-BG roslaunch --wait agent agent_node.launch
    dt-exec-BG roslaunch --wait car_interface default.launch veh:=VEHICLE_NAME
    dt-exec-BG roslaunch --wait agent lane_following_headless.launch veh:=VEHICLE_NAME
    wait_for_first_camera_frame VEHICLE_NAME
    wait_for_lane_following_mode VEHICLE_NAME
    copy-ros-logs

The launcher does the following:

- Starts `roscore` and the Duckiematrix bridge node.
- Launches `car_interface/default.launch` for the active vehicle.
- Launches `solution/launch/lane_following_headless.launch`, which wraps the standard `duckietown_demos` lane-following graph without visualization.
- Waits for the first camera frame and for the FSM services to be fully ready before switching to `LANE_FOLLOWING`.
- Applies optional lane-controller parameter overrides from `LANE_CONTROLLER_*` environment variables.

That wait-for-ready logic is important on `ente`, because the FSM can advertise `set_state` before all controlled nodes have finished initializing.


### `solution/`

The repository keeps the same top-level structure as the ROS template, including a local ROS package tree under `solution/`.

Because `/code/solution/devel/setup.bash` is sourced after the base catkin workspace, a package that you place in `solution/` with the same name as one in `dt-core` will override the base implementation at runtime. That is the main customization hook for this baseline.


## Local evaluation {#duckietown-baseline-local-workflow}

For rapid iteration, run the same submission image locally against the LF evaluator:

    $ dts challenges evaluate --challenge aido-LF-sim-validation

This is the fastest way to verify calibration changes, launcher changes, and package overrides before submitting to the public `ente` deployment.

### How to improve your submission {#duckietown-baseline-improve}

A good way to get started is to copy one of the packages that participates in lane following from `dt-core` into `solution/` and modify it there. The most consequential nodes in this baseline are:

- `ground_projection_node`
- `lane_filter_node`
- `lane_controller_node`

Because the local workspace is sourced last, the modified package in `solution/` will override the base one automatically.

If you would like to add a new package and node that includes a functionality not already run by `lane_following.launch` or you would like to change the connectivity of interfaces of these nodes, then you will also need:

 - to write your own launch file that launches your node and the remaining base nodes you still want to use;
 - to modify `launchers/run_and_start.sh` so it starts your new launch file.
 

### Other Possibly Useful Utilities {#duckietown-baseline-other-utilities}

The usual ROS command-line tools remain useful in the evaluator container. In particular:

- `rostopic list`
- `rostopic echo`
- `rosservice info`
- `rosparam get`

Those are the same primitives used by the baseline launcher to wait for camera frames, FSM readiness, and lane-controller parameters.

