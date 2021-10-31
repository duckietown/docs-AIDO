# Challenge `LFV` {#challenge-LFV status=ready}

The second challenge of the *AI Driving Olympics* is "lane following with dynamic vehicles" (`LFV`).
This challenge is an extension of Challenge `LF` to include additional rules of the road and other moving vehicles and static obstacles.

<div figure-id="fig:lane-following-vehicles" figure-caption="A Duckiebot doing lane following with other vehicles.">
  <img src="lfv-mixed-dbs.jpg" style='width:100%;height:auto'/>
</div>

Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown. Due to interactions with other Duckiebots, a successful solution will likely not be completely \emph{reactive}. 



## `LFV` in Simulation {#challenge-aido2_lfv status=ready}

The current versions of the lane following with vehicles in simulation are `aido2-LFV-sim-testing` and `aido2-LF-sim-validation`. These two challenges are identical except for the output that you are allowed to see. In the case of `testing` you will be able to see performance of your agent ([](#fig:submission-output-lfv))  and you will be able to download the logs and artifacts. 

<div figure-id="fig:submission-output-lfv" figure-caption="Visual output for a LFV submission.">
  <img src="lfv-output.png" style='width:100%;height:auto'/>
</div>

## Templates and Baselines {status=ready}

To get started, try one of the existing [templates](#part:embodied), which are
minimal setups that do random things but are functions, or the [baselines](#part:embodied-strategies)
which are instantiations of the templates that implement some algorithms, but probably not in an optimal 
way. Many of the past AI-DO winners are in the baseline solutions.


### `aido-LFV-sim-testing` Details {#aido-LFV-sim-testing status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-testing)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-testing/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-testing/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->

The details for "experiment manager", "simulator", and "scenario maker" parameters may be of interest and are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFV-sim-testing) (Under "Details").

### `aido-LFV-sim-validation` Details {#aido-LFV-sim-validation status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-sim-validation/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->


## `LFV` in the Duckietown Autolab {#challenge-aido_lfv-real status=ready}

The current version of the lane following real robot challenge is  `aido-LFV-real-validation`. 

Note that to test the performance of your agent on the real robot yourself, you can follow
[the instructions to run your agent on your Duckiebot](#challenge-LF_duckiebot)


### `aido-LFV-real-validation` Details {#aido-LFV-real-validation status=ready}

- [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-real-validation)
- [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-real-validation/leaderboard)
- [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido-LFV-real-validation/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->





 



 
