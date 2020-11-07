# The Duckietown Platform {#embodied_tasks status=ready}



This section focuses on the physical platform used for the embodied individual robotic challenges.
<!--as outlined in the [challenge overview](#challenge_overview).-->

For examples of Duckiebot driving see [a set of demo videos of Duckiebots driving in Duckietown](+opmanual_duckiebot#demos).

The actual embodied challenges will be described in more detail in [LF](#challenge-LF), [LFV_multi](#challenge-LFV_multi), [LFP](#challenge-LFP). <!--, [NAVV](#nav_v).-->
Note that the sequence challenges was chosen to gradually increase the difficulty of challenges by extending previous challenge solutions to more general situations.

<minitoc />

## The Duckietown Platform {#aido-duckietown status=ready}

There are three main parts in our system with which the participants will interact:

1. **Simulation and training** environment, which allows to test in simulation before trying on the real robots.

2. **Duckietown Autolabs** in which to try the code in controlled and reproducible conditions.

3. **Physical Duckietown platform**: miniature vision-based vehicles and cities in which the vehicles drive. The robot hardware and environment are rigorously specified, which makes the development extremely repeatable (For an example of this see ["Duckietown specifications"](+opmanual_duckietown#dt-ops-appearance-specifications). If you have a Duckiebot then you will want to refer to the [Duckiebot manual](+opmanual_duckiebot#assembling-duckiebot-db18). If you would like to acquire a Duckiebot please go to [get.duckietown.org](https://get.duckietown.org/).



## Duckiebots and Duckietowns

We briefly describe the physical Duckietown platform, which comprises  autonomous vehicles (*Duckiebots*) and a customizable model urban environment (*Duckietown*).

### The Duckiebot {#robot}

Duckiebots are designed with the objectives of affordability, modularity and ease of construction. They are equipped with: a front viewing camera with $160$ deg fish-eye lens capable of streaming $640\times480$ resolution images reliably at $30$ fps, and wheel encoders on the motors.

*Actuation* is provided through two DC motors that independently drive the front wheels (differential drive configuration), while the rear end of the Duckiebot is equipped with a passive omnidirectional wheel.

All the *computation* is done onboard on a Raspberry Pi 3B+ computer, equipped with a quad Core 1.4 GHz, 64 bit CPU and 1 GB of RAM.

We will support other configurations for the purposes of deploying neural networks onto the robots.

<!-- More details in section [computational substrate](#computation). -->

*Power* is provided by a $10000$ mAh battery which provides several hours ($>5$) of operation.

### The Duckietown {#environment}

Duckietowns are modular, structured environments built on two layers: the *road* and the *signal* layers ([](#fig:duckietown-environment)). Detailed specifications can be found [here](+opmanual_duckietown#dt-ops-appearance-specifications).


There are six well defined *road segments*: straight, left and right $90$ deg turns, 3-way intersection, 4-way intersection, and empty tile. Each is built on individual tiles, and their interlocking enables customizability of city sizes and topographies. The appearance specifications detail the color and size of the lines as well as the geometry of the roads.

The signal layer comprises of street signs and traffic lights. *Street signs* enable global localization (knowing where they are within a predefined map) of Duckiebots in the city and interpretation of intersection topologies. They are defined as the union of an AprilTag \cite{AprilTags} in addition to the typical road sign symbol. Their size, height and relative positioning with respect to the road are specified. Many signs are supported, including intersection type (3- or 4-way), stop signs, road names, and pedestrian crossings.


<figure>
    <figcaption figure-id="fig:duckietown-environment">The Duckietown environment is rigorously defined at road and signal level. When the appearance specifications are met, Duckiebots are guaranteed to navigate cities of any topology.</figcaption>
    <img style='width:20em' src="EnvironmentSpecification.png"/>
</figure>


### Simulation

We  provide a cloud simulation environment for training.

In a way similar to the last DARPA Robotics Challenge, we  use the simulation
as a first screening of the participants. It will be necessary for the code to run in simulation
to gain access to the Autolabs. In particular we emphasize that Duckiebots should not crash in simulation since a similar behavior may be disruptive to the physical Duckietown.

Simulation environments for each of the individual challenges will be provided as Docker containers with clearly specified APIs. The baseline solutions for each challenge will be provided as separate containers. When both containers (the simulation and corresponding solution) are loaded and configured correctly, the simulation will effectively replace the real robot(s). A proposed solution can be uploaded to our cloud servers, at which point it will be automatically run against our pristine version of the simulation environment (on a cluster) and a score will be assigned and returned to the uploader.

Examples of the simulators provided are shown in [](#fig:maxsim).
The left panel shows a lightweight simulator with low-level timing control built on OpenGL. This simulator is also integrated with the OpenAI Gym environment for reinforcement learning agent training. An API for designing reward functions or tweaking domain randomization will be provided.

<figure>
    <figcaption figure-id="fig:maxsim">Lightweight simulation environment for training and development</figcaption>
    <img style='width:15em' src="maxsim.png"/>
</figure>



### Duckieton Autolabs

<figure>
    <figcaption figure-id="fig:robotarium">The robotarium at ETH Zürich</figcaption>
    <img style='width:15em' src="ethz-autolab.jpg"/>
</figure>


<!-- Currently the Georgia Tech robotarium has about 300 users. The users are able to submit programs that guide the movements of a swarm of robots. The system queues the requests, runs the programs, then sends the results, before resetting the robots to the initial state for the next user. Because there is no human intervention required, and the robot self-charge, the robotarium can run continuously. -->

The idea of a robotarium (contraction of *robot* and *aquarium*) was conceived at Georgia Tech \cite{robotarium}. 
The use of a robotarium has two main advantages:

1. Convenience: It allows convenient access to a complete robot setup.
2. Reproducibility: It allows for multiple people to run the experiments in repeatable controlled conditions.


The Duckietown robotariums will be built in the following  institutions:

1. ETH Zürich;
3. University of Montréal;
4. TTI Chicago.


### Computational substrate available {#computation status=draft}

For the competition we will several options for computational power.

1. The "purist" computational substrate option: the only computation available is the Raspberry PI 3B+ processor on board.

2. The images are streamed to a basestation with a powerful GPU. This will increase computational power but also increase the latency in the control loop.

## Interface {status=draft}

Each *Duckiebot* has the following interface to the physical or simulated *Duckietown*.

**Inputs:**

A Duckiebot has a front-facing camera and encoders on each motor as described [here](#robot).

- Thus the Duckiebot receives images (both in simulation and physical reality) of resolution $640\times480$ reliably at a rate of $30$ fps.

<!-- - For the [navigation](#nav_v) challenge a map of current Duckietown will additionally be communicated. -->




<!-- TODO: JZ: Example picture, Example map -->

**Outputs:**

<!-- TODO: JZ: check this. provide figure with output commands -->

<!-- steering command and speed command -->

The Duckiebot interacts with the world through its actuators, its wheel motors.


- The output of the Duckiebot both in simulation and reality are its two motor command signals.
