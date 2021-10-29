# Challenge `LFVI-full-multi`  {#challenge-LFVI-multi-stateful status=ready}

The third challenge of the *AI Driving Olympics* is "lane following with dynamic vehicles and intersections" (`LFVI`).
This challenge is an extension of Challenge `LF` to include map configurations that are not just loops but now contain intersections which must be negotiated. 



<figure figure-id="fig:lane-following-vehicles-intersections-LFVI">
    <figcaption>A Duckiebot doing lane following with other vehicles and intersections</figcaption>
    <img style='width:15em' src="yield.jpg"/>
</figure>


Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown, but now it must also successfully navigate intersections. Due to interactions with other Duckiebots, a successful solution almost certainly not be completely \emph{reactive}. 

<!-- * This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition repository](https://github.com/duckietown/challenge-aido_LF) -->


## `LFVI` in Simulation {#challenge-aido2_lfvi status=ready}

The current versions of the lane following with vehicles in simulation are `aido2-LFVI-sim-testing` and `aido2-LF-sim-validation`. These two challenges are identical except for the output that you are allowed to see. In the case of `testing` you will be able to see performance of your agent ([](#fig:submission-output-lfvi))  and you will be able to download the logs and artifacts. 

<figure figure-id="fig:submission-output-lfvi">
    <figcaption>Visual output for submission</figcaption>
    <img style='width:30em' src="submission-output-lfvi.png"/>
</figure>

## Templates {status=ready}

To get started, try one of the existing templates:

* The [random template](#minimal-template) is the most flexible
* The [TensorFlow template](#tensorflow-template) is the place to submit a [TensorFlow](https://www.tensorflow.org/) submission
* The [PyTorch template](#pytorch-template) is the place to submit a [PyTorch](https://pytorch.org/) submission
* The [ROS template](#ros-template) is the place to submit a submission using the [Robot Operating System](http://www.ros.org/). 

or baseline algorithms:

 - [Classical Duckietown stack](#ros-baseline),
 - [Reinforcement learning](#embodied_rl) (with PyTorch),
 - [Imitation learning from simulation](#embodied_il_sim) (with tensorflow),
 - [Imitation learning from real logs](#embodied_il_logs) (with tensorflow).


### `aido6-LFVI-sim-testing` Details {#aido2-LFVI-sim-testing status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-testing)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-testing/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-testing/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->

The details for "experiment manager", "simulator", and "scenario maker" parameters may be of interest and are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-testing) (Under "Details").

### `aido6-LFVI-sim-validation` Details {#aido2-LFVI-sim-validation status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-validation/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->


## `LFVI` in the Robotarium {#challenge-aido2_lfvi_robotarium status=draft}

Details coming soon...

