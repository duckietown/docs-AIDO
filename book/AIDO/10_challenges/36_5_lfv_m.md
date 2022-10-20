# Challenge `LFV_multi` {#challenge-LFV_multi status=draft}

This challenge is an extension of Challenge `LF` to include additional rules of the road and other moving vehicles. In this challenge your agent embodies **all* of the vehicles on the road. 

<div figure-id="fig:lane-following-vehicles-LFV_multi" figure-caption="A Duckiebot doing lane following with other vehicles. In this _multi_ variant, the submitted agent runs on all Duckiebots.">
  <img src="lfv-db19.jpg" style='width:100%;height:auto'/>
</div>

Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown. 

<!-- * This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition on the challenge server][LFV_multi]. -->


[LFV_multi]: https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation


## `LFV_multi` in Simulation {#challenge-aido_lfv_multi status=ready}

The current versions of the lane following with vehicles in simulation are `aido5-LFV_multi-sim-testing` and `aido5-LF-sim-validation`. These two challenges are identical except for the output that you are allowed to see. In the case of `testing` you will be able to see performance of your agent ([](#fig:submission-output-lfv_multi))  and you will be able to download the logs and artifacts. 

<div figure-id="fig:submission-output-lfv_multi" figure-caption="Visual output for a LFV-multi submission.">
  <img src="lfv-multi-output.png" style='width:100%;height:auto'/>
</div>

## Templates {status=draft}

To get started, try one of the existing templates:

* The [random template](#minimal-template) is the most flexible
* The [ROS template](#ros-template) is the place to submit a submission using the [Robot Operating System](http://www.ros.org/). 
* The [TensorFlow template](#tensorflow-template) is the place to submit a [TensorFlow](https://www.tensorflow.org/) submission
* The [PyTorch template](#pytorch-template) is the place to submit a [PyTorch](https://pytorch.org/) submission


or baseline algorithms:

 - [Classical Duckietown stack](#ros-baseline).


### `aido5-LFV_multi-sim-testing` Details {#aido5-LFV_multi-sim-testing status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-testing/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->

The details for "experiment manager", "simulator", and "scenario maker" parameters may be of interest and are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-testing) (Under "Details").

### `aido5-LFV_multi-sim-validation` Details {#aido5-LFV_multi-sim-validation status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido5-LFV_multi-sim-validation/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->


## `LFV_multi` in the the Duckietown Autolab {#challenge-aido5_lfv_multi_robotarium status=draft}

Details coming soon...




 



 
