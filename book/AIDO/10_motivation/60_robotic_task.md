# General problem statement {#general_problem status=draft}

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

TODO:  I think that we should reorder this. I would put these general considerations in the end and start directly with the olympics? Is that possible?

TODO: JZ: cite andrea's work on task specification

A *robotic task* consists of one or more objectives, at least one robot and an environment in which to complete the objectives.
More formally, a *robotic task* $\robotictask$ is defined by a *set of objectives* $\objective = \{\objective_1, \dots, \objective_L\}$, a *robot* $\robot$ and an *environment* $\environment$.

$$
\robotictask = \objective \times \robot \times \environment
$$

To make this more tangible, we will first provide a general overview and then define the individual parts of a *robotic task*.


\begin{center}
\begin{figure}[h!]
\centering
\includestandalone[width=0.4\textwidth]{feedback_action_selection}
\caption{A model of how information is processed by a robot $\robot$ with pose $\pose$ with objective $\objective$ operating in a scene sampled from the environment $\scene \sim \environment$ with sensor measurements $\outputy$ and control action output $u$. Both scene $\scene$ and robot $\robot$ change according to dynamics $\dynamical$ and robotic action.}
\label{fig:robotic_system}
\end{figure}
 \end{center}
 \textcolor{red}{C: $p$ is not described in the figure, is it? }

TODO: JZ: Time discretizer are still missing $y(t) \Rightarrow y_k$


## Objective $\objective$ 

What is the purpose of a robot? 

The story of any robot, any machine, starts with a task, stated explicitly or implicitly. 
Here we are going to implicitly define the task via an objective function $\objective$. 


\begin{definition}[Objective]
An objective $\objective_i$ is a function of time $t \in \timespace$, pose $\pose$ of the robot, which is generally influenced by the behavior of the robot and the state of the world the robot is currently in (the scene $\scene$). We define the tuple $\alpha = (\pose, \scene)$ of pose of the robot $\pose$ and the state of the scene $\scene$. Then

$$
J(t) := f(t, \pose, \scene) = f(t, \alpha), \qquad \forall t \in \timespace
$$

Furthermore, we specify the following characteristics for the objective $\objective$. 
\end{definition}

\begin{enumerate}
	\item $\objective \geq 0$ without loss of generality
	\item $\objective$  is monotonically increasing over time
    \item $\objective$ is time equivariant. The rule mapping $\alpha$ to $\objective$ does not change with time ($\sigma_\tau (\objective(t)) = \objective(\sigma_\tau(t))$, where $\sigma_\tau$ is the time-shift operator $\sigma_\tau \phi(t) = \phi(t-\tau)$. 
\end{enumerate}

 \textcolor{red}{C: the third characteristic is not clear to me. $\phi$ is not specified...?}

TODO: JZ:  Example profile of at least two objectives here.


**Remark:** The time-averaged derivative over a period of length $T$ of an objective $\int_t^{t+T} \frac{d \objective}{d t}$, can be regarded as equivalent to the more commonly used notion of *cost function* $C = \int_t^{t+T} \frac{d \objective}{d t}$ or *reward function* $R= - \int_t^{t+T} \frac{d \objective}{dt}$. 


To connect objectives $\objective$ mathematically to an environment $\environment$ and a robot $\robot$, we define the following *decorator* operators which are inspired by function decorators in the Python-programming language. 


A priori, an objective is not connected to an optimization problem since there is nothing to optimize over. We introduce the following notation to connect an objective $\objective$ to a set of variables $x$ over which the objective may be optimized. The music operator $\sharp$ is chosen (used to increase a tone by half a note), since the objective is "elevated" to an idealized optimization problem without constraints. 

% Define decorator here

\begin{align}
x^\sharp \{\objective \} =
\begin{aligned}
& \underset{x}{\text{minimize}}
& & \objective (x)
\end{aligned}
\end{align}

 \textcolor{red}{C: why elevated? One could argue that the original objective $\mathcal{J}$ is actually more elevated, i.e., more general.}

Similarly the objective does not by itself have real-world constraints $\constraints$ attached to it. We introduce notation to connect constraints $\constraints$ to the objective's optimization variables $x$. The music operator $\flat$ is chosen here (used to decrease a tone by half a note), since the objective is now in a more non-idealized setting with constraints. 

\begin{align}
\constraints^\flat \{\objective(x) \} =
\begin{aligned}
& \underset{x}{\text{minimize}}
& & \objective(x) \\
& \text{subject to}
& & \constraints(x)
\end{aligned}
\end{align}

We will reuse this notation to connect objective $\objective$, environment $\environment$ and robot $\robot$ into a robotic task $\robotictask$. 



## Environment $\environment$ {#environment}

The environment $\environment$ is meant to model the "world" $\universe$ a robot is acting in. To adapt this notion to the purposes of a robotic task $\robotictask$, only the part of the universe that is external to the robot is considered. 

TODO: JZ: This needs to be formalized.

\begin{definition}[Environment]
An environment $\environment$ is a distribution over all possible configurations of the world, given environmental constraints $\constraints_\environment$. An environment $\environment$ therefore defines the "universe" $\universe$, the distribution of possible world instantiations, a robot may find itself in.

$$
E = p_\environment(U \setminus \{f,h\} \setminus q | \constraints_\environment)
$$
\end{definition}

We now proceed from the general distribution $\environment$ to a specific instantiation of the distribution $\environment$, the scene $\scene$. The scene $\scene$ is a particular configuration of the environment from which it is sampled. The scene itself changes according to the dynamics depending on the current scene $\scene$ and action inputs $\action$ of robots $\robot$. 

\begin{definition}[Scene $\scene$]


\begin{align}
\scene & \sim \environment \\
\dot{\scene} &= v(\scene, \action)
\end{align}
\end{definition}


## Robot $\robot$ {#robot}

TODO: AC: a general comment here. Could we define the sets that all of these functions are acting on more specifically for each map?

Any *robotic task* is connected to at least one robot. To define this more thoroughly, we will define what a robot $\robot$ is in the following. The idea behind this is to make explicit what is generally assumed implicitly. 

\begin{definition}[Robot]
A robot $\robot$ or embodied agent is a tuple of the following constituent parts
\begin{itemize}
\item (A1) Body / configuration space $\config$ including actuators: 
$$
\dot{\pose}_t = \config(\pose, \action_t, \scene) 
$$


The robot has a body $\config$ which is defined to encompass how the robot moves within a given scene $\scene$, given its current configuration or pose $\pose$ and input commands $\action_t$. 

\item (A2) A set of sensors $\sensors$, which map 
$$
\outputy_t = \sensorproj(\pose, \scene)
$$

A robot has a set of sensors $\sensors$ which are functions which given a pose $\pose$ map characteristics of the scene $\scene$ 

\item (A3-A5) Computation $\computation$, memory $\memory$ and prior knowledge $\knowledge$
$$
\action_t = \computation(y_t, \memory, \knowledge)
$$

\item (A6) Social protocol/rules $\rules$:
Rules $\rules$ are additional objective $\objective$ which penalize actions in certain situations. 
Rules are modeled as an additional objective dimension. Rules are meant to penalize types of behavior $\actions$ in a certain context $\alpha$. 
$$
\rules = \objective(t, \alpha)
$$

\item (A7) Resource constraints (Energy, Bandwidth Actuators, Sensors, Computation, Memory) $\constraints$:
Energy and bandwidth constraints are physical constraints that cannot be violated. 



$$
\constraints = \{\action \in \actions_\constraints, \dot{\action} \in \dot{\actions}_\constraints, \outputy \in \perception_\constraints, \dot{\outputy} \in \dot{\perception}_\constraints,  \computation \in \computation_\constraints, \dot{\computation} \in \dot{\computation}_\constraints, \memory \in \constraints_\memory, \dot{\memory} \in \constraints_{\dot{\memory}} \}
$$

\end{itemize}

\end{definition}



## Robotic task

Any robotic task $\robotictask = \objective \times \robot \times \environment$ is dependent not only on the specific objectives $\objective$ given but also changes considerably in a different environment or with a different robot. This is to say that the very same task may be solved in completely different ways if either the environment or the robot changes. 

$$
\robotictask = \constraints_{\environment, \robot}^\flat \{ \robot_x^\sharp \{ \objective \} \}
$$

 

### Characterization of tasks

* Instantaneous vs. Temporal (requires memory)
* Real-time frequency of task
* Priority/hierarchy of task among tasks
* Feedforward and/or Feedback
* Feasible / Infeasible





