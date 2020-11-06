# ROS Template for `aido5-LF*` {#ros-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using the [Robot Operating System](http://www.ros.org/). It can be used as a starting point for any of the [`LF`](#lf), [`LFV_multi`](#lf_vm), and [`LFP`](#lf_p) challenges.

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Requires: That you have a basic understanding of [ROS](http://www.ros.org/).

Result:  You make a submission to all of the `LF*` challenges and can view their status and output.

</div>


## Quickstart 


### Clone the [template repo](https://github.com/duckietown/challenge-aido_LF-template-ros):

    $ git clone -b daffy git@github.com:duckietown/challenge-aido_LF-template-ros.git

### Change into the directory:

    $ cd challenge-aido-LF-template-ros

### Test the submission:

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

The submission consists of all of the basic files that required for a [basic submission](#minimal-template). Below we will highlight the specifics with respect to this template. 


There are also a few other **new** files in this submission, which we will explain:

    rosagent.py
    template.launch
    

### Dockerfile

TODO: Update

If you build your own `catkin_ws` inside this template, you would probably also want to compile with `catkin_make` (this is done for you in [](#ros-baseline))

Also note that in this Dockerfile we are not copying the entire directory over, instead we are copying files individually (this is actually more efficient). So if you add new files that you are using, you will have to add additional `COPY` commands. 

### solution.py

We use the `roslaunch` API to start a launch file `template.launch`:

```python
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_path = os.path.join(os.getcwd(), "template.launch")
    self.launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
    self.launch.start()
```

We instantiate a `ROSAgent()` and this becomes the object that handles interfacing with the ROS interface. This includes the publishing of imagery to ROS:

```python
    self.agent._publish_img(obs)
    self.agent._publish_info()
```

and the setting of actions:

```python
    pwm_left, pwm_right = self.agent.action
```



### `template.launch` {#ros-template-template-launch}

Launch files in ROS are used to start many nodes at once and also load parameters and many other things. See [here](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch) for a tutorial on `roslaunch`. In this case our launch file, `template.launch` is basically empty:

```xml
<launch>
    <!-- Nothing inside - will just start a ROSMaster -->
</launch>
```

As a result the `roscore` application will get run and nothing else. In order to make this solution better you will need to launch more nodes! (The launch file `tf_slim.launch` in [](#ros-baseline) could be a good starting point).


### `rosagent.py` {#ros-template-rosagent-py}

`rosagent.py` sets up a class that can be used to interface with the rest of the ROS stack. It is for all intents and purposes a fully functional ROS node except that it isn't launched through ROS launched it is instantiated in code. 

The two main functions are:
- `_publish_img(observation)`, which takes the observation from the environment, and publishes it to the topic that you specify in the constructor of the `ROSAgent` (You will want to replace the placeholder with the topic name that your node is expecting the image to come through on)
- `_ik_action_cb(msg)`, listens on the inverse kinematic action topic, and assigns it to `self.action`. Because we are using wheel velocities as the action, but the output of many Duckietown nodes is a `Twist2DStamped` message, we need a way to convert this back into the actions we are interested in. The `self.action` is the action executed in the simulation at a set rate.

To improve this submission, you can look at the [Duckietown software stack](https://github.com/duckietown/Software) as described in [](#ros-baseline).
