# Embodied individual robot tasks {#embodied_tasks status=ready}


There are three embodied individual robotic tasks.


  * [Lane following (LF)](#lf): Control of a Duckiebot to drive on the right lane on streets within Duckietown without other moving Duckiebots present.



  * [Lane following + vehicles (LFV)](#lf_v): Control of a Duckiebot to drive on the right lane on streets within Duckietown with other Duckiebots and duckies present.


  <!-- * [Navigation + vehicles (NAVV)](#nav_v): Navigation task of a Duckiebot to drive from point $A$ to point $B$ within Duckietown while following the rules of the road and while other Duckiebots are likewise driving in the road. -->

----------------------------


This section focuses on the infrastructure and background of the embodied individual robotic tasks as outlined in the [task overview](#task_overview).

For examples of Duckiebot driving see [a set of demo videos of Duckiebots driving in Duckietown](+opmanual_duckiebot#demos).

The actual embodied tasks will be described in more detail in [LF](#lf), [LF](#lf_v). <!--, [NAVV](#nav_v).-->
Note that the sequence tasks was chosen to gradually increase the difficulty of tasks by extending previous task solutions to more general situations.

## Platform

There are three main parts in our system with which the participants will interact:

1. The **physical Duckietown platform** ([](#fig:duckietown_nice)): miniature vision-based vehicles and cities in which the vehicles drive. This is an inexpensive setup (\$100/robot). The robot hardware and environment are rigorously specified, which makes the development extremely repeatable (For an example of this see ["Duckietown specifications"](http://docs.duckietown.org/opmanual_duckietown/out/duckietown_specs.html).

2. A **cloud simulation** and training environment, which allows to test in simulation before trying on the real robots.

3. **Remote "robotariums"** in which to try the code in controlled and reproducible conditions.

### Device$\rightarrow$cloud$\rightarrow$device deployment pipeline

We expect most participants to choose to build their own Duckietown and Duckiebots. All of the equipment needed to do so is available at a reasonable cost, and all of the instructions are provided on the project website.

The cloud simulation is provided as an alternative for development.

The cloud simulation also serves as a selection mechanism to access the remote robotarium.

The robotariums are needed to enable reproducible testing in controlled conditions. The robotarium scores are valid scores for the leader board and they are used for the final selection of which code will be run at the live competition at NIPS.

For winning the competitions, the only valid scores
are the scores obtained at the live competition. The participants will not need to be physically at NIPS --- they can participate remotely by submitting a Docker container, which will be run for them following standardized procedures.

## The physical Duckietown platform

We briefly describe the physical Duckietown platform, which comprises  autonomous vehicles (*Duckiebots*) and a customizable model urban environment (*Duckietown*).

### Robot {#robot}

Duckiebots are designed with the objectives of affordability, modularity and ease of construction. They are equipped with only one *sensor*: a front viewing camera with $160$ deg fish-eye lens capable of streaming $640\times480$ resolution images reliably at $30$ fps.

*Actuation* is provided through two DC motors that independently drive the front wheels (differential drive configuration), while the rear end of the Duckiebot is equipped with a passive omnidirectional wheel.

All the *computation* is done onboard on a Raspberry Pi 3+ computer, equipped with a quad Core 1.4 GHz, 64 bit CPU and 1 GB of RAM.

We will support other configurations for the purposes of deploying neural networks onto the robots.
More details in section [computational substrate](#computation).

*Power* is provided by a $10000$ mAh battery which provides several hours ($>5$) of operation.

### Environment {#environment}

Duckietowns are modular, structured environments built on two layers: the *road* and the *signal* layers ([](#fig:duckietown-environment)). Detailed specifications can be found [here](http://docs.duckietown.org/opmanual_duckietown/out/duckietown_specs.html).


There are six well defined *road segments*: straight, left and right $90$ deg turns, 3-way intersection, 4-way intersection, and empty tile. Each is built on individual tiles, and their interlocking enables customizability of city sizes and topographies. The appearance specifications detail the color and size of the lines as well as the geometry of the roads.

The signal layer comprises of street signs and traffic lights. *Street signs* enable global localization (knowing where they are within a predefined map) of Duckiebots in the city and interpretation of intersection topologies. They are defined as the union of an AprilTag \cite{AprilTags} in addition to the typical road sign symbol. Their size, height and relative positioning with respect to the road are specified. Many signs are supported, including intersection type (3- or 4-way), stop signs, road names, and pedestrian crossings.

Intersections are always separated by at least 2 tiles to avoid traffic jams of Duckiebots reaching into an intersection.

*Traffic lights*  provide a centralized solution for intersection coordination, encoding signals in different LED blinking frequencies. They are equipped with an overhead camera, with field of view of one tile in every direction from the intersection.

<div figure-id="fig:duckietown-environment">
<img src="images/EnvironmentSpecification.png" style="width: 90%"/>
<figcaption>The Duckietown environment is rigorously defined at road and signal level. When the appearance specifications are met, Duckiebots are guaranteed to navigate cities of any topology.</figcaption>
</div>


### Cloud simulation

Additionally, we will provide a cloud simulation environment for training.

In a way similar to the last DARPA Robotics Challenge, we will use the simulation
as a first screening of the participants. It will be necessary for the code to run in simulation
to gain access to the robotariums. In particular we emphasize that Duckiebots should not crash in simulation since a similar behavior may be disruptive to the physical Duckietown.

Simulation environments for each of the individual challenges will be provided as Docker containers with clearly specified APIs. The baseline solutions for each challenge will be provided as separate containers. When both containers (the simulation and corresponding solution) are loaded and configured correctly, the simulation will effectively replace the real robot(s). A proposed solution can be uploaded to our cloud servers, at which point it will be automatically run against our pristine version of the simulation environment (on a cluster) and a score will be assigned and returned to the uploader.

Examples of the simulators provided are shown in [](#fig:maxsim).
The left panel shows a lightweight simulator with low-level timing control built on OpenGL. This simulator is also integrated with the OpenAI Gym environment for reinforcement learning agent training. An API for designing reward functions or tweaking domain randomization will be provided. The simulator in the right panel is built on Gazebo and has much more high fidelity physics models for high-level control algorithm testing and tuning and will be used to test algorithms for the navigation tasks.

<div figure-id="fig:simulations">
<img src="images/maxsim.png" figure-id="fig:maxsim" style="width:50%"/>
<img src="images/gazebo_sim.png" figure-id="fig:gazebo_sim" style="width:90%"/>
<figcaption>
We have developed various ways of simulating the Duckiebots sensors,
navigating the tradeoff of simulation speed vs output accuracy.
</figcaption>
</div>

For the [autonomous mobility-on-demand](#amod) (AMoD) task, in which the AI must respond to ride requests and allocate existing cars to each task, we will develop a standard neural-networks-friendly data representation.

### Robotariums

<div figure-id="fig:robotarium">
<img src="images/robotarium.jpg" style="width:90%"/>
<figcaption>The Robotarium currently operating at Georgia Tech. It contains 30 robots. It uses an external motion capture system. About 300 users use the Robotarium
to conduct remote experiments. Two similar installations are under construction outside of Georgia Tech. }
</figcaption>
</div>


The idea of a robotarium (contraction of *robot* and *aquarium*) was conceived at Georgia Tech \cite{robotarium}. Currently the Georgia Tech robotarium has about 300 users. The users are able to submit programs that guide the movements of a swarm of robots. The system queues the requests, runs the programs, then sends the results, before resetting the robots to the initial state for the next user. Because there is no human intervention required, and the robot self-charge, the robotarium can run continuously.

The use of a robotarium has two advantages:

1. Convenience: It allows convenient access to a complete robot setup.
2. Reproducibility: It allows for multiple people to run the experiments in repeatable controlled conditions.

-----------------------
The Duckietown robotariums will be built in five institutions:

1. At ETH Zürich. The projected size is sufficient to allocate 20 robots continuously running (20 robots on the road + 20 robots in charging stations).
2.  At National Chiao Tung University, Taiwan. The size will be similar to the ETH Zürich installation.
3. At the University of Montréal. The size is to be determined; it will likely be smaller than Zürich and Taiwan.


These robotariums will remain available after the competition ends, for follow-up editions,
as well as for regular research activities.

### Computational substrate available {#computation}

For the competition we will two options for computational power.

1. The first option is the "purist" computational substrate option: the only computation available is the Raspberry PI 3 processor on board.

2. The second option is the "non-purist" option, where additionally a Movidius computation stick may be used to run more computation intensive algorithms. The baseline solutions we provide using conventional methods run in real time using the Raspberry PI processor only.



## Interface

Each *Duckiebot* has the following interface to the physical or simulated *Duckietown*.

**Inputs:**

A Duckiebot has a *single*  front-facing sensor, a camera as described [here](#robot).

- Thus the Duckiebot receives images (both in simulation and physical reality) of resolution $640\times480$ reliably at a rate of $30$ fps.

<!-- - For the [navigation](#nav_v) task a map of current Duckietown will additionally be communicated. -->




<!-- TODO: JZ: Example picture, Example map -->

**Outputs:**

<!-- TODO: JZ: check this. provide figure with output commands -->

<!-- steering command and speed command -->

The Duckiebot interacts with the world through its actuators, its wheel motors.


- The output of the Duckiebot both in simulation and reality are its motor command signals.



## Related work

Vision-based solutions for autononomus driving were independently developed in Europe, Japan and the US \cite{overview_autonomous_vision}.

### Lane following

In 1979, Tsugawa et al. \cite{japan_self_driving} developed a first application of pattern matching to drive a car within 30 km/h.

Already in 1987, Dickmanns and Zapp developed vision-based driving algorithms based on recursive estimation and were able to drive a car on structured roads at high speeds \cite{autonomous_germany}.

Using machine learning for lane following dates back to 1989, where a neural network was trained to drive the Carnegie Mellon autonomous navigation test vehicle \cite{cmu_self_driving_original}. Similarly, the company Nvidia demonstrated that also in modern time an approach similar to \cite{cmu_self_driving_original} can yield interesting results \cite{nvidia_autonomous}.

Driving within Duckietowns has been likewise been explored by the creators of Duckietown \cite{paull2017duckietown} where they describe implemented model-based solutions for autonomous driving specifically in the Duckietown environment.

### Navigation

A recent review of planning for autonomous cars both traditional as well as learning-based approaches to navigation and planning are discussed \cite{schwarting2018planning}. As an example of more recent learning approach to navigation, an end-to-end navigation neural network was trained in simulation and tested in real-world office environments \cite{Pfeiffer2017FromRobots}.
