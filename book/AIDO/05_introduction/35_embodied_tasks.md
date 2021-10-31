# The Duckietown Platform {#embodied_tasks status=ready}

The [Duckietown platform](https://www.duckietown.org/platform) has many components.

This section focuses on the physical platform used for the embodied robotic challenges.

<!--as outlined in the [challenge overview](#challenge_overview).-->

For examples of Duckiebot driving see [a set of demo videos of Duckiebots driving in Duckietown](+opmanual_duckiebot#demos).


The actual embodied challenges will be described in more detail in [LF](#challenge-LF), [LFV](#challenge-LFV), [LFI](#challenge-LFI). 

Note: the sequence of the challenges was chosen to gradually increase the difficulty, by extending previous challenge solutions to more general situations. We recommend you tackle the challenges in this same order.

<minitoc />

## The Duckietown Platform {#aido-duckietown status=ready}

There are three main parts of the platform with which you will interact:

1. **Simulation and training** environment, which allows testing in simulation before trying on the real robots.

2. **Duckietown Autolabs** in which to try the code in controlled and reproducible conditions.

3. **Physical Duckietown platform**: miniature autonomous vehicles and smart-cities in which the vehicles drive. The [Duckiebots](+opmanual_duckiebot#duckiebot-configurations) (robot hardware) and Duckietown (environment) are [rigorously specified](+opmanual_duckietown#dt-ops-appearance-specifications), which makes the development extremely repeatable. If you have a Duckiebot you can refer to the [Duckiebot operational manual](+opmanual_duckiebot#book) for step-by-step instructions on how to assemble, maintain, calibrate and operate your robot. If you would like to acquire a Duckiebot please go to [the Duckietown project store](https://get.duckietown.com/). 

The Duckiebots officially supported for AI-DO 6 (2021) are the [`DB21` Duckiebots](https://get.duckietown.com/collections/dt-robots/products/duckiebot-db21-m). We recommend you [build your Duckietowns](https://docs.duckietown.org/daffy/opmanual_duckietown/out/index.html) according to the specifications, too. The necessary materials can be sourced locally pretty much globally - but if you want compliant "one-click" AI-DO kits for each challenge you can get them from here:

- [`LF` AI-DO 6 hardware kit](https://get.duckietown.com/collections/ai-do-kits/products/ai-do-lane-following-lf-challenge-kit)
- [`LFV` AI-DO 6 hardware kit](https://get.duckietown.com/collections/ai-do-kits/products/ai-do-lane-following-with-vehicles-lfv-challenge-kit)
- [`LFI` AI-DO 6 hardware kit](https://get.duckietown.com/collections/starter-kits/products/db-mooc-kit)

For any questions regarding Duckietown hardware you can reach out to `hardware@duckietown.com`.

## Duckiebots and Duckietowns
We briefly describe the physical Duckietown platform, which comprises  autonomous vehicles (*Duckiebots*) and a customizable model urban environment (*Duckietown*).

### The Duckiebot {#robot}

Duckiebots are designed with the objectives of affordability, modularity and ease of construction. They are equipped with: a front viewing camera with 160 degrees fish-eye lens capable of streaming $640\times480$ resolution images reliably at 30 fps, and wheel encoders on the motors. `DB21` Duckiebots are equipped with IMUs and front facing time of flight sensors too.

*Actuation* is provided through two DC motors that independently drive the front wheels (differential drive configuration), while the rear end of the Duckiebot is equipped with a passive omnidirectional wheel.

All the *computation* is done onboard on a:
- `DB19`: [Raspberry Pi 3B+ computer](https://www.raspberrypi.com/products/raspberry-pi-3-model-b-plus/), 
- `DB21`: [Jetson Nano 2 GB](https://developer.nvidia.com/embedded/jetson-nano-2gb-developer-kit) (`DB21M`) or [Jetson Nano 4 GB](https://developer.nvidia.com/embedded/jetson-nano) (`DB21J`).

<!-- More details in section [computational substrate](#computation). -->

*Power* is provided by a $10000$ mAh [Duckiebattery](+opmanual_duckiebot##db-opmanual-preliminaries-electronics) which provides several hours of operation.

### The Duckietown {#environment}

Duckietowns are modular, structured environments built on two layers: the *road* and the *signal* layers ([](#fig:duckietown-environment)). Detailed specifications can be found [here](+opmanual_duckietown#dt-ops-appearance-specifications).


There are six well-defined *road segments*: straight, left and right 90 deg turns, 3-way intersection, 4-way intersection, and empty tile. Each is built on individual tiles, and their interlocking enables customizable city sizes and topographies. The appearance specifications detail the color and size of the lines as well as the geometry of the roads.

The signal layer comprises street signs and traffic lights. *Street signs* enable global localization (knowing where they are within a predefined map) of Duckiebots in the city and interpretation of intersection topologies. They are defined as the union of an AprilTag \cite{AprilTags} in addition to the typical road sign symbol. Their size, height and relative positioning with respect to the road are specified. Many signs are supported, including intersection type (3- or 4-way), stop signs, road names, and pedestrian crossings.


<figure>
    <figcaption figure-id="fig:duckietown-environment">The Duckietown environment is rigorously defined at road and signal level. When the appearance specifications are met, Duckiebots are guaranteed to navigate cities of any topology.</figcaption>
    <img style='width:20em' src="EnvironmentSpecification.png"/>
</figure>


### Simulation

We provide a cloud simulation environment for training.

In a way similar to the last DARPA Robotics Challenge, we use the simulation
as a first screening of the participant's submissions. It will be necessary for the submitted agent code to run in simulation and be sufficiently performant to gain access to the Autolabs. 

Simulation environments for each of the individual challenges are provided as Docker containers with clearly specified APIs. The baseline solutions for each challenge is provided as separate containers. When both containers (the simulation and corresponding solution) are loaded and configured correctly, the simulation will effectively replace the real robot(s). A proposed solution can be uploaded to our cloud servers, at which point it will be automatically run against our pristine version of the simulation environment (on a cluster) and a score will be assigned and returned to the participant.

Examples of the simulators provided are shown on the [Duckietown Challenges server](https://challenges.duckietown.org/v4/). E.g., here is a [`LF` evaluated submission example](https://challenges.duckietown.org/v4/humans/submissions/13502) from AI-DO 5. 

This simulator is also integrated with the OpenAI Gym environment for reinforcement learning agent training. An API for designing reward functions or tweaking domain randomization will be provided.

<!--

<figure>
    <figcaption figure-id="fig:maxsim">Lightweight simulation environment for training and development</figcaption>
    <img style='width:15em' src="maxsim.png"/>
</figure>

Examples of submissions and evaluations are publicly available on the [Duckietown Challenges server](https://challenges.duckietown.org/v4/). Here is a [`LF` evaluated submission example](https://challenges.duckietown.org/v4/humans/submissions/13502) from AI-DO 5. 

-->

### Duckietown Autolabs

<figure>
    <figcaption figure-id="fig:robotarium">The Duckietown Autolab at ETH Zürich</figcaption>
    <img style='width:30em' src="ethz-autolab-white.jpg"/>
</figure>


<!-- Currently the Georgia Tech robotarium has about 300 users. The users are able to submit programs that guide the movements of a swarm of robots. The system queues the requests, runs the programs, then sends the results, before resetting the robots to the initial state for the next user. Because there is no human intervention required, and the robot self-charge, the robotarium can run continuously. -->

The idea of an Autolab is inspired by Georgia Tech's Robotarium (contraction of *robot* and *aquarium*) \cite{robotarium}. 

The use of an Autolab has two main advantages:

1. Convenience: It allows convenient access to a complete robot setup.
2. Reproducibility: It allows for multiple people to run the experiments in repeatable controlled conditions.

<!--
Duckietown Autolabs are operational in the following institutions:

1. ETH Zürich;
2. University of Montréal;
3. TTI Chicago.
-->

You can find detailed information on Duckietown Autolabs in our paper: [Integrated Benchmarking and Design for Reproducible and Accessible Evaluation of Robotic Agents](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9341677&casa_token=BrfRTcJC5BEAAAAA:r0v4pMqg6q-vyxUW7rQHjw-d2Az4wli6aFBNciRfC8tB1pkDayRySGUsSs9OQv4cGs5rQbmY8lM&tag=1).  


If you would like to cite Duckietown Autolabs, please use: 

```
@INPROCEEDINGS{tani2020duckienet,
  author={Tani, Jacopo and Daniele, Andrea F. and Bernasconi, Gianmarco and Camus, Amaury and Petrov, Aleksandar and Courchesne, Anthony and Mehta, Bhairav and Suri, Rohit and Zaluska, Tomasz and Walter, Matthew R. and Frazzoli, Emilio and Paull, Liam and Censi, Andrea},
  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Integrated Benchmarking and Design for Reproducible and Accessible Evaluation of Robotic Agents}, 
  year={2020},
  volume={},
  number={},
  pages={6229-6236},
  doi={10.1109/IROS45743.2020.9341677}}
```


### Computational substrate available {#computation status=draft}

For the competition we will several options for computational power.

1. The "purist" computational substrate option: where processing is done onboard Duckiebots. 

2. The images are streamed to a base-station with a powerful GPU. This will increase computational power but also increase the latency in the control loop.

## Interface {status=draft}

Each *Duckiebot* has the following interface to the physical or simulated *Duckietown*.

**Inputs:**

A Duckiebot has a front-facing camera and encoders on each motor as described [here](#robot).

- Thus, the Duckiebot receives images (both in simulation and physical reality) of resolution $640\times480$ reliably at a rate of $30$ fps.

<!-- - For the [navigation](#nav_v) challenge a map of current Duckietown will additionally be communicated. -->




<!-- TODO: JZ: Example picture, Example map -->

**Outputs:**

<!-- TODO: JZ: check this. provide figure with output commands -->

<!-- steering command and speed command -->

The Duckiebot interacts with the world through its actuators, its wheel motors.


- The output of the Duckiebot both in simulation and reality are its two motor command signals.
