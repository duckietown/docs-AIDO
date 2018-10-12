# Challenge `aido1_lf1` {#challenge-aido1_lf1-v3 status=ready}

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





 



 
