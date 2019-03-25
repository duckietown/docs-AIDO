# Challenge `aido1_LF1`: Lane Following {#challenge-aido1_lf1 status=ready}

The current version of the lane following challenge is `aido1_LF1_r3-v3`.
The lane following challenge tests an agent using a simulator.

Here is how the simulator looks:

<video autoplay="1" controls="1" loop="1" style="border: solid 1px black" width="320">
  <source src="http://duckietown-ai-driving-olympics-1.s3.amazonaws.com/v3/frankfurt/by-value/sha256/db648be4473470451c3ff8131f5c9a96849c812ab30db88ea48e61e089c60405" type="video/mp4"/>
</video>
 
* Check out the [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido1_LF1_r3-v3/leaderboard) to see who's currently winning!
 

* This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition repository](https://github.com/duckietown/challenge-aido1_lf1)

To get started, try on of the existing templates:

* The [random template](#challenge-aido1_lf1-template-random) is the most flexible
* The [Tensorflow template](#tensorflow-template) is the place to submit a [tensorflow](https://www.tensorflow.org/) submission
* The [Pytorch template](#pytorch-template) is the place to submit a [pytorch](https://pytorch.org/) submission
* The [ROS baseline](#ros-template) is the place to submit a submission using the [Robot Operating System](http://www.ros.org/). 

Interaction protocol: [`aido1_remote3-v3`](#aido1_remote3-v3)

## Simulator parameters

The Duckietown Gym Parameters are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido1_LF1_r3-v3#step1-simulation).

In particular:

```yaml
DTG_EPISODES: 5
DTG_STEPS_PER_EPISODE: 500
DTG_MAP: loop_obstacles
```

## Metrics 

Metrics are [described here](https://challenges.duckietown.org/v4/humans/challenges/aido1_LF1_r3-v3#scoring).





 



 
