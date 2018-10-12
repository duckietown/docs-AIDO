# Challenge `aido1_lf1` {#challenge-aido1_lf1 status=ready}

The challenge `aido1_lf1` tests the agent using a simulator.

Here is how it looks like:

<video autoplay="1" controls="1" loop="1" style="border: solid 1px black" width="320">
  <source src="http://duckietown-ai-driving-olympics-1.s3.amazonaws.com/v3/frankfurt/by-value/sha256/db648be4473470451c3ff8131f5c9a96849c812ab30db88ea48e61e089c60405" type="video/mp4"/>
</video>

## Leaderboard

* [Leaderboard](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1-v3/leaderboard)

## Repositories

* [Challenge definition repository](https://github.com/duckietown/challenge-aido1_lf1)
* [Solution template using Tensorflow](https://github.com/duckietown/challenge-aido1_LF1-template-tensorflow)

## Interaction protocol: `aido1_remote1` {#protocol-aido1_remote1-v3}

* Communication using `slimremote` protocol.
* Camera images are undistorted.
* The camera resolution is 160x120.

## Simulator parameters

The Gym Parameters are [available here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1-v3#step1-simulation).

In particular:

```yaml
DTG_EPISODES: 5
DTG_STEPS_PER_EPISODE: 500
DTG_MAP: loop_obstacles
```

## Metrics

The only metric is the average Gym Reward.

Metrics are [described here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1-v3#scoring).





 



 
