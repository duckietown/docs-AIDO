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

# Fleet-level social tasks {#social_tasks status=beta}

This section provides a brief introduction to the fleet-level social tasks "Fleet management" and "Autonomous Mobility-on-Demand". The subsequent two tasks are offered as social tasks in the AI Driving Olympics.

  * [Fleet management (FM)](#nav_v): Task to control a small fleet of Duckiebots within Duckietown to pick up a set of virtual customers and drive them to a destination point.


  * [Autonomous Mobility-on-Demand (AMoD)](#amod): Task to control the movement of a fleet of autonomous vehicles in a simulated city to pick up customers and drive them to their destinations.

----------------------------

The fleet-level tasks aim to work at a higher level of abstraction than the previous embodied robotic tasks [LF](#lf), [LFV](#lf_v), [NAVV](#nav_v). Crucially instead of focusing on single robots, now a fleet of robots should be controlled.

Picture a city with several already autonomous vehicles. The question now posed at the fleet-level is how to best control vehicles to serve customers.  


The actual social tasks will be described in more detail in ["Fleet management"](#fleet_manag) and ["Autonomous Mobility-on-Demand"](#amod). Note that the sequence tasks was chosen to gradually increase the difficulty of tasks by extending previous task solutions to more general situations.

## Further details

Since the fleet-level tasks differ in some points in terms of the used platform and evaluation, detailed information is presented separately in ["Fleet management"](#fleet_manag) and ["Autonomous Mobility-on-Demand"](#amod).


## Related Work

Fleet management is a field of research which is recently has received increase attention. This section briefly and non-exhaustively presents related work to the fleet management and AMoDeus challenge.

In \cite{horl2017fleet} several autonomous mobility-on-demand control algorithms were compared to a simulation scenario for the city of Zurich, Switzerland. It was shown
that a simple load-balancing heuristic reaches peak mean waiting times of 5 min with 10’000
vehicles, while more advanced control policies achieve the same performance with less than 9’000 vehicles.


A case study for Berlin presented in \cite{bischoff2016simulation} simulates city-wide replacement of transportation systems with robotic taxis in Berlin. The results are obtained with a heuristic algorithm and for a large number of agents.

In \cite{spieser2014toward} the authors use analytic results to estimate who  many robotic taxis could satisfy  the entire transportation demand of the country of Singapore. They estimate that $\approx 300,000$ taxis would be sufficient to serve demand with acceptable waiting times.

In \cite{zhang2016control} the relation of AMoD systems and queueing systems is detailed and a method to compute the fleet sizes required for certain levels of performance is presented.
