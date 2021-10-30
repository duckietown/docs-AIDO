# Challenge `LFVI-full-multi`  {#challenge-LFVI-multi-stateful status=ready}

The fourth challenge of the *AI Driving Olympics* is "lane following with dynamic vehicles and intersections" (`LFVI`).
This challenge is an extension of Challenge `LF` to include map configurations that are not just 
loops but now contain intersections which must be negotiated. 
Your agent will control all of the Duckiebots in the map. We make things somewhat simpler by providing
directly the state information of the Duckiebots. As a result, this challenge will only be evaluated
in simulation


<figure figure-id="fig:lane-following-vehicles-intersections-LFVI">
    <figcaption>A Duckiebot doing lane following with other vehicles and intersections</figcaption>
    <img style='width:15em' src="yield.jpg"/>
</figure>


Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown, but now it must also successfully navigate intersections. Due to interactions with other Duckiebots, a successful solution almost certainly not be completely \emph{reactive}. 

<!-- * This challenge uses the Duckietown challenge infrastructure. The precise definition of the challenge is in the [challenge definition repository](https://github.com/duckietown/challenge-aido_LF) -->


## `LFVI_multi_full` in Simulation {#challenge-aido_lfvi status=ready}

The current versions of the lane following with vehicles in simulation are `aido-LFVI_multi-sim-testing` 
and `aido-LFVI_multi-sim-validation`. These two challenges are identical except for the output that you are 
allowed to see. In the case of `testing` you will be able to see performance of your agent 
([](#fig:submission-output-lfvi))  and you will be able to download the logs and artifacts. 

<figure figure-id="fig:submission-output-lfvi">
    <figcaption>Visual output for submission</figcaption>
    <img style='width:30em' src="submission-output-lfvi.png"/>
</figure>

## Templates {status=ready}

## Templates and Baselines {status=ready}

To get started, try one of the existing [templates](#part:embodied), which are
minimal setups that do random things but are functions, or the [baselines](#part:embodied-strategies)
which are instantiations of the templates that implement some algorithms, but probably not in an optimal 
way. Many of the past AI-DO winners are in the baseline solutions.

Note that in the case of this challenge you will need to update the protocol that is used. 

TODO: provide more details. 

You may also look at the [minimal agent with full state information](https://github.com/duckietown/challenge-aido_LF-minimal-agent-full)
for an example of how to do this.


### `aido-LFVI_multi-sim-testing` Details {#aido-LFVI_multi-sim-testing status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-testing)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-testing/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-testing/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->

The details for "experiment manager", "simulator", and "scenario maker" parameters may be of interest and are [available here](https://challenges.duckietown.org/v4/humans/challenges/aido2-LFVI-sim-testing) (Under "Details").

### `aido-LFVI_multi-sim-validation` Details {#aido-LFVI_multi-sim-validation status=ready}

 - [Challenge overview](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-validation)
 - [Leaderboard](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-validation/leaderboard)
 - [All submissions](https://challenges.duckietown.org/v4/humans/challenges/aido-LFVI_multi-sim-validation/submissions)


<!-- Interaction protocol: [`aido2_db18_agent-z2`](#aido2_db18_agent-z2) -->


