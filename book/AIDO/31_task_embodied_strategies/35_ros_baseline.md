# Classical Duckietown Baseline (ROS) {#ros-baseline status=ready}

This section describes the basic procedure for making a submission using the [Robot Operating System](http://www.ros.org/) and the  [Duckietown software stack](https://github.com/duckietown/Software) .

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template) and you understand how it works.

Requires: You already know something about ROS.

Result: You could win the AI-DO!

</div>

## Quickstart

### Clone this [repo](https://github.com/duckietown/challenge-aido_LF-baseline-duckietown)

    $ git clone -b daffy git://github.com/duckietown/challenge-aido_LF-baseline-duckietown.git --recursive

### Change into the `3_submit` directory

    $ cd challenge-aido_LF-baseline-duckietown/3_submit
    
### Evaluate your submission

Either locally with 

    $ dts challenges evaluate
    
Or make an official submission when you are ready with 

    $ dts challenges submit
    

## Local Development Workflow


We have provided a local setup where you can qualitatively evaluate the performance of your agent against the simulator. 

### Updating submodules

First make sure that your submodules are up to date with the `daffy` branch:

    $ git submodule init
    $ git submodule update
    $ git submodule foreach "(git checkout daffy; git pull)"


### Change into the `1_develop` directory

    $ cd challenge-aido_LF-baseline-duckietown/1_develop


### Run locally

    $ docker-compose build
    $ docker-compose up

Note: To run this, you will need to have `docker-compose` installed on your local machine, as unlike the AIDO submissions, this will emulate both the server and agent all on your local machine. Follow instructions [here](https://docs.docker.com/compose/install/) to install.


You should see that three containers are starting:

```
Starting 1_devlelop_sim_1   ... done
Starting 1_devlelop_novnc_1        ... done
Starting 1_devlelop_lanefollow_1 ... done
```

To start your agent you will need a terminal in the `1_devlelop_lanefollow_1` container. You can do that one of two ways:

#### In a new terminal 
   
    $ docker exec -it 1_devlelop_lanefollow_1 /bin/bash

#### In the browser with Jupyter notebook

Open your browser and copy and paste in the url that was produced in the output to the terminal when you ran `docker-compose`. It should look something like:

    http://127.0.0.1:8888/?token={SOME_LONG_TOKEN}
    
Then click on the dropdown for "New" in the top right and click "Terminal". 

### Building the software

You can build the software by running in the terminal:

    $ catkin build --workspace catkin_ws
    
Note: the build artifacts are preserved outside docker so you can shut down docker and restart it and you would not need to rebuild (unless you made changes to the source code of course). You can also edit the source files with any editor you like on your local machine and then build them inside the container since the files are [mounted](https://docs.docker.com/storage/volumes/).

You will then want to do 

    $ source catkin_ws/devel/setup.bash

Note: this source command is run by default when you open a new terminal so you don't need to worry about it if you didn't just rebuild the workspace


### Launch your submission

Now you can go ahead and launch your submission. For example, the default submission provided in the repo is `lf_slim.launch`. Start it with

    $ roslaunch custom/lf_slim.launch
    
Note that you can also launch other launch files in the duckietown stack. Check out the `duckietown_demos` folder in the `dt-core` repository for other launch files that might be interesting. 

### Viewing and Debugging with noVNC

Navigate your browser to `http://localhost:6901/vnc.html`. Click `Connect`. The password is `quackquack`. This will open up a fully functional desktop inside your browser. In the top left click `Applications` and choose `Terminal Emulator` from the list. In the terminal run

    $ rqt_image_view

and in the dropdown menu select the topic `default/camera_node/image/compressed`. You should see the agent moving in the simulator.

<figure>
    <figcaption>The simulator running inside the noVNC browser</figcaption>
    <img style='width:8em' src="sim_in_novnc.png"/>
</figure>

Note: there are many other tools in [rqt](http://wiki.ros.org/rqt) that you might find useful. To access them run `rqt` and then choose a plugin. 

Another great visualization tool is:

    $ rviz

Click the "Add" button in the bottom left and then the `By Topic` tab. You might find the `/duckiebot_visualizer/segment_list_markers/` and the filterer version as well as the `/lane_pose_visualizer_node/lane_pose_markers` particularly interesting.

<figure>
    <figcaption>Rviz running inside the noVNC browser</figcaption>
    <img style='width:8em' src="rviz_in_novnc.png"/>
</figure>

Note: You will see more  outputs (e.g., the `image_with_lines`) if you  set the verbose flag to true by typing `rosparam set /default/line_detector_node/verbose true` from any terminal. 


### How to Improve your Submission {#ros-workflow}


You will notice one main difference as compared to the [ROS template](#ros-template) is that the launch file argument in the `solution.py` code now points to `lf_slim.launch`, which launches a slimmed-down version of the [Duckietown Lane Following code](https://github.com/duckietown/dt-core). The action and image topics are both adjusted to match the inputs and outputs of the original stack.

#### lf_slim.launch

Compared to `template.launch` in [](#ros-template), the launch file here actually runs some nodes. These are nodes that are defined in the [Duckietown Software repo](https://github.com/duckietown/dt-core). The code that is being run is all included as "submodules" in this repository. 

The nodes which are getting launched are the following:

 - [line_detector_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/line_detector): Used to detect the lines in the image.
 - [ground_projection_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/ground_projection): Used to project the lines onto the ground plane using the camera extrinsic calibration.
 - [lane_filter_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/lane_filter): Used to take the ground projected line segments and estimate the Duckiebot's position and orientation in the lane
 - [lane_controller_node](https://github.com/duckietown/Software/tree/master18/catkin_ws/src/10-lane-control/lane_control): Used to take the estimate of the robot and generate a reference linear and angular velocities for the Duckiebot

A good way to get started could be modify or make your own version of the nodes that are being launched in `lf_slim.launch`.

For testing purposes you can go ahead and modify the code that's included. However, ultimately you will have to write your own nodes and packages. 

### Create a repo for your packages the template repo

Now we are going to discuss how you might build a new node and incorporate it into the structure of an existing launch file. 

Follow the instructions [here](https://github.com/duckietown/template-ros-core) to create a new repo for your packages. Clone this repo inside the folder `challenge-aido_LF-baseline-duckietown/catkin_ws/src`. You can add new packages inside the `packages` folder of that repo. 

Any packages that you put in there will be built when you run:

    $ catkin build --workspace catkin_ws

in the notebook terminal. 

If you have added a new launch file then you can launch it from the notebook terminal with:

    $ roslaunch ![your_package_name] ![your_launch_file_name]


A good way to build your own launch file would be to use the provided launchfile, `lf_slim.launch`, as a template. You may also look at `lane_following.launch` inside the `duckietown_demos` package in `dt-core`. This includes a `master.launch` file and turns nodes on and off using the `args` that correspond to the new nodes that you don't want to run and then add your own `include` or `node` tags launching code after. You'll want to create a new launchfile, rather than editing the original `lane_following.launch`. 


## Updating your Submission

If you've followed either of our local workflows, you'll notice that the `1_develop` directory you've been working in no longer contains any of the submission files. The easiest way to do take your local submission and submit it to AIDO is to copy the following files over into the `3_submit` directory of this repository.

- `lf_slim.launch`
- Any nodes you created 

In addition, within `1_develop/Dockerfile`, you will want to copy and paste the code regarding the `CUSTOM CATKIN_WS` and copy it into the corresponding location into `3_submit/Dockerfile`. 
