
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

**TL;DR** 
There are two containers:

- `gym-duckietown-server`, runs the actual gym environment
- `gym-duckietown-agent`, hold the agent code (i.e. your code)

We provide scripts for pulling, building and launching both containers.

## Prerequisites

Currently we support development on Mac and Linux. We are working on supporting Windows soon.

Please make sure you have the following installed on your PC:

- [Docker CE](https://docs.docker.com/install/) for running the containers
- [Docker Compose](https://docs.docker.com/compose/install/#install-compose) for starting both `server` and `agent` and make them communicate with each other in one command
- Some Python code editor for modifying the agent. We'd recommend [PyCharm](https://www.jetbrains.com/pycharm/) or [Atom](https://atom.io/)
- [Git](https://git-scm.com/downloads) for pulling the repositoy.

## Install

Point your browser to this URL: https://github.com/duckietown/gym-duckietown-agent, click the `Fork` button (top right) and then click your Github account when it asks you where you want to fork to.

TODO: insert fork image here

Now you should see the page of your forked repo. Now let's clone that new repo. Click the green `Clone or Download` button (right side), copy the URL, open a terminal and clone your repo

    git clone https://github.com/[YOUR GITHUB USERNAME HERE]/gym-duckietown-agent.git
    
That's it for the "installation" part. :)

## Run






