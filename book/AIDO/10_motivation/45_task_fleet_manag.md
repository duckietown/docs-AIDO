$$
\newcommand{\AC}[1]{{\color{blue}AC: #1}}
\newcommand{\JZ}[1]{{\color{olive}JZ: #1}}
\newcommand{\fix}{\marginpar{FIX}}
\newcommand{\new}{\marginpar{NEW}}
% Robot:
\newcommand{\dynamical}{\mathcal{D}}
\newcommand{\robot}{\mathcal{R}} % Robot
\newcommand{\config}{\mathcal{Q}} % Configuration space (of robot)
\newcommand{\sensors}{\{z\}} % Sensor set
\newcommand{\bandwidth}{\mathcal{B}}
\newcommand{\computation}{\mathcal{C}}
\newcommand{\memory}{\mathcal{M}}
\newcommand{\actuators}{\mathcal{A}}
\newcommand{\knowledge}{\mathcal{K}}
\newcommand{\perception}{P}
\newcommand{\control}{U}
\newcommand{\actions}{\mathcal{U}}
% Robot mathematics
\newcommand{\operator}{T}
% Groups:
\newcommand{\groups}{G}
%\newcommand{\group}{g}
\newcommand{\groupalgebra}{\mathfrak{g}}
% Scene space
\newcommand{\timespace}{\mathbb{T}}
\newcommand{\environment}{E}
\newcommand{\scene}{\xi}
\newcommand{\scenespace}{\Xi}
\newcommand{\universe}{U}
% Sensor space
\newcommand{\sensor}{\zeta}
\newcommand{\sensorproj}{z}
\newcommand{\sensorspace}{Z}
\newcommand{\projection}{\pi}
\newcommand{\projectionspace}{\Pi}
\newcommand{\viewport}{v}
\newcommand{\viewportspace}{\mathcal{V}}
% Data space
\newcommand{\dataspace}{\mathcal{X}}
\newcommand{\data}{x}
\newcommand{\dataproj}{\phi}
\newcommand{\datakernel}{\psi}
% Output space
\newcommand{\outputy}{y}
\newcommand{\outputspace}{\mathcal{Y}}
% Task space
\newcommand{\task}{T}
\newcommand{\taskspace}{\mathcal{T}}
\newcommand{\objective}{\mathcal{J}}
\newcommand{\robotictask}{RT}
\newcommand{\rules}{\Phi}
\newcommand{\constraints}{\Lambda}
% Action space
\newcommand{\action}{u}
\newcommand{\actionspace}{\mathcal{U}}
\newcommand{\nuisance}{\nu}
% Other characteristics / symbols
\newcommand{\place}{\eta}
\newcommand{\image}{I}
\newcommand{\noise}{n}
\newcommand{\pose}{p}
\newcommand{\shape}{S}
\newcommand{\albedo}{\rho}
% Information theory
\newcommand{\information}{\mathcal{I}}
\newcommand{\expectation}{\mathbb{E}}
% Optimization
\newcommand{\loss}{L}
$$

# Task: Fleet management in Duckietown (FM) {#fleet_manag status=beta}

## Description of task
Task FM further scales task NAV. Now you are asked to control multiple Duckiebots at once. You will be provided with location pairs $(A,B,t)$ provided at time $t$. Performance will be measured based on how quickly these location pairs can be served with the available fleet of Duckiebots. The main idea is that the submitted code now operates at a higher level of abstraction that can choose to neglect lower level controls.

This scenario is meant to resemble a taxi-service. Customer request to go from location "A" to location "B" arrive sequentially and need to be served in an intelligent manner.



<div figure-id="fig:Autolab_map">
 <img src="images/Autolab_map.png" style="width: 90%"/>
 <figcaption>Map of a Duckietown provided to illustrate Task NAV and FM. Best viewed in color.</figcaption>
</div>

## Platform

There are two main parts in our system with which the participants will interact:

1. A **cloud simulation** and training environment, which allows to test in simulation before trying on the real robots.
2. **Remote "robotariums"** in which to try the code in controlled and reproducible conditions.

### Device$\rightarrow$cloud$\rightarrow$device deployment pipeline

The cloud simulation serves as a selection mechanism to access the remote robotarium.

The robotariums are needed to enable reproducible testing in controlled conditions. The robotarium scores are valid scores for the leader board and they are used for the final selection of which code will be run at the live competition at NIPS.

For winning the competitions, the only valid scores
are the scores obtained at the live competition. The participants will not need to be physically at NIPS --- they can participate remotely by submitting a Docker container, which will be run for them following standardized procedures.

### The physical Duckietown platform

We briefly describe the physical Duckietown platform, which comprises  autonomous vehicles (*Duckiebots*) and a customizable model urban environment (*Duckietown*).

#### Computational substrate available  

For this part of the computation, computation will be performed in real-time on the cloud. Calculated commands will be sent to Duckiebots.

#### Environment

Similar to the embodied tasks, the fleet management task takes place in Duckietown ([](#fig:duckietown-environment).

#### Cloud simulation

We will provide a cloud simulation environment for training based on the AMoDeus simulator \cite{amodeus}.


## Performance metrics



### Performance objective

As performance objective on task FM, we again calculate the expected time to go from point A to point B. The difference to task NAV is that now multiple trips $(A,B)$ may be active at the same time. Likewise, multiple Duckiebots are now available to service the additional requests. To reliably evaluate the metric, multiple pairs of points A, B will be sampled.


$$
\objective_{speed}(t) = \expectation_{A,B}[T_{A \to B}]
$$


### Rule objectives

Similar to tasks LF and LFV, the rules of the road have to be observed.


### Comfort objectives

Additionally, driving "comfortably" will be measured. Comfort in this case is trying to promote smoothness of the vehicle behavior by penalizing changes in the input commands.

$$
\objective_{CM} = \expectation[ ||\Delta u_k ||_{1} ] \approx \frac{1}{N} \sum_k^N |\Delta u_k|
$$

Maximal waiting time:

$$
\objective_{WT} = ||T_{wait} ||_\infty]
$$

The robot $\robot$ used in this task is a Duckiebot as described in [](#robot). The environment of the task is Duckietown as described in [](#environment). Different to task 1 and 2, the input to the Duckiebot now also includes a map of Duckietown.

TODO: determine map of Duckietown


## Interface
TODO:

### Deployment technique

We will use Docker containers to package, deploy, and run the applications on the physical Duckietown platform as well as on the cloud for simulation. Base Docker container images will be provided and distributed via [Docker HUB][dockerhub].

[dockerhub]: https://hub.docker.com/r/duckietown/

A *Master* server will be used to collect and queue all submitted programs ([](#fig:dockerflow)) The *simulation evaluation agents* will execute each queued program as they become available. Submissions that pass the simulation environment will be queued for execution in the robotariums.

<div figure-id="fig:dockerflow2">
\input{dockerflow.tex}
<figcaption>Submission, Deployment, and Execution Flow
</figcaption>
</div>


### Submission of entries

The website will allow for submission of entries by
submitting a Docker container name, or an IPFS hash of a Docker container image to be downloaded.

Scripts will be provided for creating the container image in a conforming way.

The system will schedule to run the code on the cloud on the challenges selected by the user, and, if simulations pass, on the robotariums.

Participants can submit entries as many times as they would like.

Access control policies are to be implemented, should certain participants monopolize the computational resources available.

### Simulators

Simulation code will be available as open source for everybody to use on computers that they control.

Amazon AWS will make available cloud resources to run the cloud simulations and the cloud learning. The access to these resources might be rationed if the utilization exceeds the projections.

### Robotarium test and validation

If there are $n$ robotariums available, $n-1$ robotariums can be used for training and testing, while 1 robotarium is used for validation.

When an experiment is run in a training/testing robotarium, the participants receive, in addition to the score, detailed feedback, including logs, telemetry, videos from external cameras, etc.

The sensory data generated by the robots is continuously recorded and becomes available immediately to the entire community.

When an experiment is run in a validation robotarium, the only output to the user is the test score and minimal statistics (number of collisions, number of rule violations, etc.)

### Leaderboards

After each run in a robotarium, the participants can see the metrics statistics in the competition website.

Participants can choose whether to make public any of the results.

Leaderboards are reset at the beginning of October 2018.
