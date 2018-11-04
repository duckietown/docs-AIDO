# ROS Template for `aido1_LF1` {#ros-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using the [Robot Operating System](http://www.ros.org/).

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Requires: That you have a basic understanding of [ROS](http://www.ros.org/).

Result: You make a submission and see your entry on the [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).

</div>


## Quickstart 


### Clone the [ repo](https://github.com/duckietown/challenge-aido1_LF1-template-ros):

    $ git clone git@github.com:duckietown/challenge-aido1_LF1-template-ros.git

### Enter the repo:

    $ cd challenge-aido1-LF1-template-ros

### Test the submission:

Either make a submission with:

    $ dts challenges submit


Or, run local evaluation with:

    $ dts challenges evaluate
    

### Verify the submission:

You should be able to see your submission [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).

For more tips on how to make use of the existing Duckietown codebase try [](#ros-baseline). The important part of the code is the following block:


## Anatomy of the submission

Just like [any submission](#challenge-aido1_lf1-template-random), your submission contains the following files:

    submission.yaml
    Dockerfile
    requirements.txt
    solution.py

Here we will only highlight difference from the agent template, so for a basic understanding of the use of these files, please see [any submission](#challenge-aido1_lf1-template-random).

There are also a few other **new** files in this submission, which we will explain:

    rosagent.py
    template.launch
    

### Dockerfile

There is one main additional block in the `Dockerfile`, which is important for setting up ROS-specific things:

```docker
# Source it to add messages to path
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN echo "source /home/software/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

If you build your own `catkin_ws` inside this template, you would probably also want to compile with `catkin_make` (this is done for you in [](#ros-baseline))

Also note that in this Dockerfile we are not copying the entire directory over, instead we are copying files individually (this is actually more efficient). So if you add new files that you are using, you will have to add additional `COPY` commands. 

### solution.py

Just like the [random-template](#challenge-aido1_lf1-template-random), `solution.py` uses the OpenAI gym interface for interacting with the server environment. This done through the line: 

```python
observation, reward, done, info = env.step(action)
```

where an action is sent to the simulator and an observation (and some other things) is returned.

In this case, we are using the ROS interface to define the action. 


```python
    # Now, initialize the ROS stuff here:
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    roslaunch_path = os.path.join(os.getcwd(), "template.launch")
    launch = roslaunch.parent.ROSLaunchParent(uuid, [roslaunch_path])
    launch.start()
 
    # Start the ROSAgent, which handles publishing images and subscribing to action 
    agent = ROSAgent()
    r = rospy.Rate(15)

    # While there are no signal of completion (simulation done)
    # we run the predictions for a number of episodes, don't worry, we have the control on this part
    while not rospy.is_shutdown():
        # we passe the observation to our model, and we get an action in return
        # we tell the environment to perform this action and we get some info back in OpenAI Gym style
        
        # To trigger the lane following pipeline, we publish the image 
        # and camera_infos to the correct topics defined in rosagent
        agent._publish_img(observation)
        agent._publish_info()

        # The action is updated inside of agent by other nodes asynchronously
        action = agent.action
```

The first thing that happens is we launch the launchfile `template.launch` (See [](#ros-template-template-launch) below for details about this file)

Next we initialize the ROSagent (See [](#ros-template-rosagent-py) below for details).

Now in the infinite loop we first publish the camera image that we got from the server to ROS with the line

```python
agent._publish_img(observation)
```

and then we grab the most recent action calculated by the agent and send it as our next action in the line

```python
action = agent.action
```

In this solution template, you will build up your ROS-based solution to subscribe to this image and to publish the actions. This template provides the interface between your ROS code and the OpenAI gym Duckietown server. 


### template.launch {#ros-template-template-launch}

Launch files in ROS are used to start many nodes at once and also load parameters and many other things. See [here](http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch) for a tutorial on `roslaunch`. In this case our launch file, `template.launch` is basically empty:

```xml
<launch>
    <!-- Nothing inside - will just start a ROSMaster -->
</launch>
```

As a result the `roscore` application will get run and nothing else. In order to make this solution better you will need to launch more nodes! (The launch file `tf_slim.launch` in [](#ros-baseline) could be a good starting point).


### rosagent.py {#ros-template-rosagent-py}

rosagent.py sets up a class that can be used to interface with the rest of the ROS stack. It is for all intents and purposes a fully functional ROS node except that it isn't launched through ROS launched it is instantiated in code. 

It sets up a Publisher and Subscriber. The publisher sends out the camera imagery received from the Duckietown OpenAI gym server, and the subscriber receives wheel commands. 

At present, the function `_TEMPLATE_action_publisher` gets called which chooses a random action:

```python
    def _TEMPLATE_action_publisher(self):
        """
        TODO: You need to change this!
        Random action publisher - so your submission does something
        """
        
        vl = np.random.random()
        vr = np.random.random()
        self.action = np.array([vl, vr])
```

You will need to change this if you want the submission to get better. 

One option could be to use the [Duckietown software stack](https://github.com/duckietown/Software) as described in [](#ros-baseline).
