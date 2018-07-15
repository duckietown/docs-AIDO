
# Gym-Duckietown AIDO tutorial {#gym-tutorial status=draft}

This is a tutorial on how to make a submission for the **LF** 
(lane following on a closed course) and **LFV** (lane following 
with dynamic obstacles) AIDO challenge. This challenge uses
the gym-duckietown environment from https://github.com/duckietown/gym-duckietown
but packs it up in a docker container. 

To this end the `gym-duckietown` environment lives in one
container called `gym-duckietown-server`. The code for the agent that 
**you** as the participant have to modify as well as the code for 
communicating with the `-server` container live in a different
repository called `gym-duckietown-agent`. We will guide you
through the process of installing everything, forking this repo, 
modifying the code and making a submission.

## Prerequisites



