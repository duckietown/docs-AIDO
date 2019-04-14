# Challenge `LFV` {#lf_v status=ready}

The second challenge of the *AI Driving Olympics* is "lane following with dynamic vehicles" (`LFV`).
This challenge is an extension of Challenge `LF` to include additional rules of the road and other moving vehicles and static obstacles.



<figure figure-id="fig:lane-following-vehicles">
    <figcaption>A Duckiebot doing lane following with other vehicles</figcaption>
    <img style='width:15em' src="lane_following_v.jpg"/>
</figure>


Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown. Due to interactions with other Duckiebots, a successful solution will likely not be completely \emph{reactive}. 

* This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition repository](https://github.com/duckietown/challenge-aido_LF)


## `LFV` in Simulation {#challenge-aido2_lfv status=ready}

The current versions of the lane following with vehicles in simulation are `aido2-LFV-sim-testing` and `aido2-LF-sim-validation`. These two challenges are identical except for the output that you are allowed to see. In the case of `testing` you will be able to see performance of your agent ([](#fig:submission-output-lfv))  and you will be able to download the logs and artifacts. 

<figure figure-id="fig:submission-output-lfv">
    <figcaption>Visual output for submission</figcaption>
    <img style='width:30em' src="submission-output-lfv.png"/>
</figure>

To get started, try on of the existing templates:

* The [random template](#minimal-template) is the most flexible
* The [Tensorflow template](#tensorflow-template) is the place to submit a [tensorflow](https://www.tensorflow.org/) submission
* The [Pytorch template](#pytorch-template) is the place to submit a [pytorch](https://pytorch.org/) submission
* The [ROS template](#ros-template) is the place to submit a submission using the [Robot Operating System](http://www.ros.org/). 

or baseline algorithms:

 - [Classical Duckietown stack](#ros-baseline),
 - [Reinforcement learning](#embodied_rl) (with PyTorch),
 - [Imitation learning from simulation](#embodied_il_sim) (with tensorflow),
 - [Imitation learning from real logs](#embodied_il_logs) (with tensorflow).


### `aido2-LFV-sim-testing` Details {#aido2-LFV-sim-testing status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-testing)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-testing/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-testing/submissions)


Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2)

The details for "experiment manager", "simulator", and "scenario maker" parameters may be of interest and are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-testing) (Under "Details").

### `aido2-LFV-sim-validation` Details {#aido2-LFV-sim-validation status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-validation/submissions)


Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2)


## `LFV` in the Robotarium {#challenge-aido2_lfv_robotarium status=ready}

Details coming soon...




 



 
