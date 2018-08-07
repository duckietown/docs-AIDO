# Performance objective {#performance status=beta}

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

## Lane following (LF / LFV) {#performance_lf}
As a performance indicator for both the "lane following task" and the "lane following task with other dynamic vehicles", we choose the speed $v(t)$ along the road (not perpendicular to it) over time of the Duckiebot. This measures the moved distance along the road per episode, where we fix the time length of an episode. This encourages both faster driving as well as algorithms with lower latency. An *episode* is used to mean running the code from a particular initial configuration.


$$
\objective_{P-LF(V)}(t) = \int_{0}^{t} - v(t) dt
$$

The integral of speed is defined over the traveled distance of an episode up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

## Navigation (NAVV) {#performance_navv}
Similarly, for the "navigation with dynamic vehicles task" (NAVV), we choose the time it takes to go from point $A$ to point $B$ within a Duckietown map as performance indicator. A trip from $A$ to $B$ is *active* as soon as it is received as long as it has not been completed.


This is formalized in the equation and integral below.
$$
\objective_{P-NAVV}(t)  =  \int_{0}^{t}  \mathbb{I}_{AB-active} dt
$$

The indicator function $\mathbb{I}_{AB-active}$ is $1$ if a trip is *active* and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

<!-- ## Fleet management (FM) {#performance_fm}

As performance objective on task FM, we calculate the sum of trip times to go from $A_{i}$ to $B_{i}$. This generalizes the objective from task NAVV to multiple trips. The difference to task NAVV is that now multiple trips $(A_{i},B_{i})$ may be active at the same time. A trip is *active* as soon as it is requested and as long as it has not been completed. Likewise, multiple Duckiebots are now available to service the additional requests. To reliably evaluate the metric, multiple pairs of points A, B will be sampled at different time points within an episode.

$$
\objective_{P-FM}(t) =  \sum_i \int_{0}^{t} \mathbb{I}_{i-active} dt
$$

The indicator function $\mathbb{I}_{i-active}$ is $1$ if a trip is \emph{active} and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode. -->

## Autonomous mobility on demand (AMoD) {#performance_amod}


An AMoD system needs to provide the highest possible service level in terms of journey times and wait times, while ensuring maximum fleet efficiency.
We have two scoring metrics representing these goals in a simplified manner. In order to normalize their contributions, we supply a baseline case $\mathcal{B}$

We introduce the following variables:

\begin{align*}
&\alpha_1 = \frac{1}{2}& &\textrm{weight for efficiency}& \\
&\alpha_2 = \frac{1}{2}& &\textrm{weight for waiting times}& \\
&\alpha_3 = \frac{1}{2}& &\textrm{weight for fleet size}& \\
&d_T& &\textrm{total distance driven by fleet}& \\
&d_E& &\textrm{empty distance driven by fleet}& \\
&R \in \mathbb{N}^+& &\textrm{total number of requests in the scenario}&\\
&w_i \in \mathbb{R}& &\textrm{waiting time of request $i$}&\\
&w_{i,B} \in \mathbb{R}& &\textrm{waiting time of request $i$ in the $\mathcal{B}$ case}&\\
&N \in \mathbb{N}^+& &\textrm{number of robotic taxis used}&\\
&N_B \in \mathbb{N}^+& &\textrm{number of robotic taxis used in the $\mathcal{B}$ case}&\\
\end{align*}

The first performance metric is for cases when the same number of vehicles as in the benchmark case $N_\mathcal{B}$ is used:

$$
\objective_{P-AMOD-1} = 0.5 \cdot \frac{d_E}{d_T} + 0.5 \cdot \frac{\sum_{i=1}^K w_i}{\sum_{i=1}^K w_{i,\mathcal{B}}}
$$

The second performance metric allows the designer to reduce the number of vehicles, if possible, or increase it if deemed useful:

$$
\objective_{P-AMOD-2} = \objective_{AMOD-1} + 0.5 \cdot \frac{N}{N_B}
$$

For the AMoD task, only a performance metric will be evaluated. Robotic taxis are assumed to already observe the rules of the road as well as drive comfortably. Through the abstraction of the provided AMoD simulation, these conditions are already enforced.
