# Challenge `LFV` {#lf_v status=ready}

The second task of the *AI Driving Olympics* is "Lane following with dynamic vehicles".
This task is an extension of Task LF to include additional rules of the road and other moving vehicles and static obstacles.

Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown. Due to interactions with other Duckiebots, the task is designed no longer completely \emph{reactive}. Intersections will be recognized and maneuvered using provided code from the organizers.


<div figure-id="fig:others1">
<img src='images/lane_following_v.jpg' figure-id="subfig:lane_following_v" style="width:90%"/>
<img src='images/in_lane_sideview.jpg' figure-id="subfig:lane_following_v_ego"  style="width:90%"/>
</div>

<div figure-id="fig:others2">
<img src='images/collision1.jpg' figure-id="subfig:collision1"  style="width:90%"/>
<img src='images/yield.jpg' figure-id="subfig:aido-yield"  style="width:90%"/>
</div>



The robot used in this task is a Duckiebot as described in [](#robot). The environment of the task is Duckietown as described in [](#environment). The main addition to the lane following task is that now obstacles may appear on the road which shall be avoided.



## Evaluation

The lane following task is evaluated on three separate objectives.

### Performance objective

The [*performance objective*](#performance_lf) measures how fast a Duckiebot moves.


### Traffic law objective

The following traffic laws apply in the lane following with dynamic vehicles task.

* [Staying in the lane](#traffic_laws_lf)
* [Collision avoidance](#traffic_laws_ac)



### Comfort objective

The following objective quantifies how "comfortable" a Duckiebot is driving.

[Comfortable driving](#comfort_embodied)


<!-- ## Submission scenario

TODO: user submits code, then what happens

how does evaluation work -->


## Challenge `aido1_LFV1`: Lane Following with Obstacles {#challenge-aido1_lfv1 status=ready}

The current version of the lane following challenge is `aido1_LFV1_r3-v3`.
The lane following challenge tests an agent using a simulator.

Here is how the simulator looks:

<video autoplay="1" controls="1" loop="1" style="border: solid 1px black" width="320">
  <source src="http://duckietown-ai-driving-olympics-1.s3.amazonaws.com/v3/frankfurt/by-value/sha256/db648be4473470451c3ff8131f5c9a96849c812ab30db88ea48e61e089c60405" type="video/mp4"/>
</video>
 
* Check out the [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido1_LF1_r3-v3/leaderboard) to see who's currently winning!
 

* This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition repository](https://github.com/duckietown/challenge-aido1_lf1)

To get started, try on of the existing templates but change the challenge name in `submission.yaml` to `aido1_LFV_r1-v3`:

* The [random template](#challenge-aido1_lf1-template-random) is the most flexible
* The [Tensorflow template](#tensorflow-template) is the place to submit a [tensorflow](https://www.tensorflow.org/) submission
* The [Pytorch template](#pytorch-template) is the place to submit a [pytorch](https://pytorch.org/) submission
* The [ROS baseline](#ros-template) is the place to submit a submission using the [Robot Operating System](http://www.ros.org/). 

Interaction protocol: [`aido1_remote3-v3`](#aido1_remote3-v3)

### Simulator parameters

The Duckietown Gym Parameters are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido1_LFV_r1-v3#step1-simulation).

In particular:

```yaml
DTG_EPISODES: 5
DTG_STEPS_PER_EPISODE: 500
DTG_MAP: loop_obstacles
```

### Metrics 

Metrics are [described here](https://challenges.duckietown.org/v4/humans/challenges/aido1_LFV_r1-v3#scoring).





 



 
