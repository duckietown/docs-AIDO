# Rules {#part:aido-rules}

Maintainer: Julian Zilly

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


## Performance metrics

Measuring performance in robotics is less clear cut and more multidimensional than traditionally encountered in machine learning settings. Nonetheless, to achieve reliable performance estimates we assess submitted code on several *episodes* with different initial settings and compute statistics on the outcomes. We denote $\objective$ to be an objective or cost function to optimize, which we evaluate for every experiment. In the following formalization, objectives are assumed to be minimized.

In the following we summarize the objectives used to quantify how well an embodied task is completed. We will produce scores in three different categories: *performance objective*, *traffic law objective* and *comfort objective*. Note that the these objectives are not merged into one single number.

### Performance objective

#### Lane following (LF / LFV)
As a performance indicator for the "lane following task" and the "lane following task with other dynamic vehicles", we choose the speed $v(t)$ along the road (not perpendicular to it) over time of the Duckiebot. This then in turn measures the moved distance per episode, where we fix the time length of an episode. This encourages both faster driving as well as algorithms with lower latency. An *episode* is used to mean running the code from a particular initial configuration.


$$
\objective_{P-LF(V)}(t) = \int_{0}^{t} - v(t) dt
$$

The integral of speed is defined over the traveled distance of an episode up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

##### Navigation (NAVV)
Similarly, for the "navigation with dynamic vehicles task" (NAVV), we choose the time it takes to go from point $A$ to point $B$ within a Duckietown map as performance indicator. A trip from $A$ to $B$ is *active* as soon as it is received as long as it has not been completed.


This is formalized in the equation and integral below.
$$
\objective_{P-NAVV}(t)  =  \int_{0}^{t}  \mathbb{I}_{AB-active} dt
$$

The indicator function $\mathbb{I}_{AB-active}$ is $1$ if a trip is *active* and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

#### Fleet management (FM)

As performance objective on task FM, we calculate the sum of trip times to go from $A_{i}$ to $B_{i}$. This generalizes the objective from task NAVV to multiple trips. The difference to task NAVV is that now multiple trips $(A_{i},B_{i})$ may be active at the same time. A trip is *active* as soon as it is requested and as long as it has not been completed. Likewise, multiple Duckiebots are now available to service the additional requests. To reliably evaluate the metric, multiple pairs of points A, B will be sampled at different time points within an episode.

$$
\objective_{P-FM}(t) =  \sum_i \int_{0}^{t} \mathbb{I}_{i-active} dt
$$

The indicator function $\mathbb{I}_{i-active}$ is $1$ if a trip is \emph{active} and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

#### Autonomous mobility on demand (AMoD)


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

### Traffic law objective

The following are a list of rule objectives the Duckiebots are supposed to abide by within Duckietown. All individual rule violations will be summarized in one overall traffic law objective $\objective_{T}$. These penalties hold for the lane following, navigation and fleet management tasks (LF, LFV, NAVV, FM).

#### Quantification of "Staying in the lane"

<div figure-id="fig:crossing_lane">
<img src="images/crossing_lane.jpg" style="width: 80%"/>
<figcaption>Picture depicting a situation in which the "staying-in-the-lane rule" applies.</figcaption>
</div>

The Duckietown traffic laws say:

"The vehicle must stay at all times in the right lane, and ideally near the center."

We quantify this as follows: let $d(t)$
be the absolute perpendicular distance of the center of mass the Duckiebot-body
from the middle of the right lane, such that $d(t)=0$ corresponds to the robot being in the center of the right lane at a given instant. While $d(t)$ stays within an acceptable range no cost is incurred. When the safety margin $d_{\text{safe}}$ is violated, cost starts accumulating proportionally to the square of $d(t)$ up to an upper bound $d_{max}$. If even this bound is violated a lump penalty $\alpha$ is incurred.

The "stay-in-lane" cost function is therefore defined as:

 $$
   \objective_{T-LF}(t) = \int_0^{T_{eps}} \begin{cases} 0  & d(t) < d_{safe} \\
     \beta d(t)^2 & d_{safe} \leq d(t) \leq d_{max} \\
   	\alpha & d(t) > d_{max}
   	\end{cases}
 $$

An example situation where a Duckiebot does not stay in the lane is shown in \ref{fig:crossing_lane}.


#### Quantification of "Stopping at red intersection line"

 The Duckietown traffic laws say:

 "Every time the vehicle arrives at an intersection with a red stop line,
 the vehicle should come to a complete stop  in front of it, before continuing."


<div figure-id="fig:intersection">
<img src="images/intersection_stop.jpg" style="width:80%"/>
<figcaption>Picture depicting a Duckiebot stopping at a red intersection line.
</figcaption>
</div>


During each intersection traversal, the vehicle is penalized by~$\gamma$ if there was not a time~$t$ when the vehicle was at rest ($v(t) = 0$) in the stopping zone defined as the rectangular area of the same width as the red line between $a$ and $b$ cm distance from the start of the stop line perpendicular to the center of mass point of the Duckiebot. This situation is demonstrated in Fig.~\ref{fig:intersection}.

The condition that the position~$p(t)$ of the center of mass of the Duckiebot
is in the stopping zone is denoted with ~$p(t) \in \mathcal{S}$.

Then we write the objective as the cumulative sum of stopping at intersection rule infractions.

 $$
  	\objective_{T-SI}(t) = \sum_{t_k} \gamma  \mathbb{I}_{\nexists t \text{ s.t. } v(t)=0 \wedge  p(t) \in S_{zone}}
 $$

Here the sum over time increments $t_k$ denote the time intervals in which this conditions is checked. The rule penalty is only applied once the Duckiebot leaves the stopping zone. Only then is it clear that it did not stop within the stopping zone.

To measure this cost, the velocities $v(t)$ are evaluated while the robot is in the stopping zone $\mathcal{S}$. %An example of a Duckiebot stopping at a red intersection line is depicted in Fig.~\ref{fig:intersection}.

#### Quantification of "Keep safety distance"

The Duckietown traffic laws say:

 "Each Duckiebot should stay at an adequate distance from the Duckiebot in front of it, on the same lane, at all times."

We quantify this rule as follows: Let $b(t)$
denote the distance between
the center of mass of the Duckiebot and the center of mass of the closest Duckiebot in front of it which is also in the same lane. Furthermore let $b_{\text{safe}}$ denote a cut-off distance after which a Duckiebot is deemed "far away". Let $\delta$ denote a scalar positive weighting factor. Then

$$
\objective_{T-SD}(t) = \int_0^t \delta \cdot \max(0,b(t)- b_{\text{safe}})^2.
$$

#### Quantification of "Avoiding collisions"

The Duckietown traffic laws say:

*At any time a Duckiebot shall not collide with another object or Duckiebot.*

<div figure-id="fig:collision">
<img src="images/collision1.jpg" style="width:80%"/>
<figcaption>Picture depicting a collision situation.
</figcaption>
</div>

The vehicle is penalized by $\nu$ if within a time a time interval of length $t_k$ $t \in [t, t+t_k)$, the distance $\ell(t)$ between the vehicle and a nearby object or other vehicle is zero or near zero. $\ell(t)$ denotes the perpendicular distance between any object and the Duckiebot rectangular surface. The collision cost objective therefore is

  \begin{align*}
 	\objective_{T-AC}(t) = \sum_{t_k} \nu \mathbb{I}_{\exists t \in [ t-t_k, t ) \ell(t) < \epsilon}
  \end{align*}

Time intervals are chosen to allow for maneuvering after collisions without incurring further costs.

An illustration of a collision is displayed in Fig.~\ref{fig:collision}.

#### Quantification of "Yielding the right of way"

The Duckietown traffic laws say:

 *Every time a Duckiebot arrives at an intersection with a road joining on the right, it needs to check whether there are other Duckiebots on the right-hand lane of the joining road. If so, these vehicles shall traverse the intersection first.*

Mathematically we accumulate penalties $\mu$ whenever the Duckiebot moves at an intersection while there is a Duckiebot (DB) on the right hand joining lane (RHL).

  $$
 	\objective_{T-YR}(t) = \sum_{t_k} \mu \mathbb{I}_{v(t) >0 \wedge \exists \text{ DB in RHL}}
  $$

<div figure-id="fig:yield">
<img src="images/yield.jpg" style="width:80%"/>
<figcaption>Picture depicting a situation in which the *yield rule* applies.
</figcaption>
</div>

The yield situation at an intersection is depicted in Fig.~\ref{fig:yield}.

#### Hierarchy of rules

To account for the relative importance of rules, the factors $\alpha, \beta, \gamma, \delta, \nu, \mu$ of the introduced rules will be weighted relatively to each other.

Letting $>$ here denote "more important than", we define the following rule hierarchy:

$$
\objective_{T-AC} > \objective_{T-SI} > \objective_{T-YR} > \objective_{T-SD} > \objective_{T-LF}
$$

I.e.:

\begin{center}Collision avoidance $>$ Stop line $>$ Yielding $>$ Safety distance $>$ Staying in the lane.
\end{center}

This constrains the factors $\alpha, \beta, \gamma, \delta, \nu, \mu$ whose exact values will be determined empirically to enforce this relative importance.

While the infractions of individual rules will be reported, as a performance indicator all rule violations are merged into one overall traffic law objective $\objective_{T}$. Let $\task$ denote a particular task, then the rule violation objective is the sum of all individual rule violations $\objective_i$ which are an element of that particular task.

$$
\objective_{T} = \sum_i \mathbb{I}_{\objective_i \in \task} \objective_{T-i},
$$

where $\mathbb{I}_{\objective_i \in \task}$ is the indicator function that is $1$ if a rule belongs to the task and $0$ otherwise.

## Comfort metric

#### Lane following and navigation (LF, LFV, NAVV)

In the single robot setting, we encourage "comfortable" driving solutions. We therefore penalize large accelerations to achieve smoother driving. This is quantified through smoothed changes in Duckiebot position $p_{bot}(t)$. Smoothing is performed by convolving the Duckiebot position $p_{bot}(t)$ with a smoothing filter $k_{smooth}$.

As a comfort metric, we measure the smoothed absolute changes in position $\Delta p_{bot}(t)$ over time.

$$
\objective_{C-LF/LFV/NAVV}(t) = \int_0^t k_{smooth} * \Delta p_{bot}(t) dt
$$

%where $M$ denotes the number of time steps \JT{throughout the whole task?}.

#### Fleet management (FM)

In the fleet management setting "customer experience" is influenced greatly by how fast and dependable a service is. If it is known that a taxi arrives quickly after ordering it, it makes the overall taxi service more convenient.

We therefore define the comfort metric as the maximal waiting time $T_{wait}$ until customer pickup. Let $T_{wait}$ denote the time beginning at the reception of a ride request until when the ride is started.  


Let $S_{\text{wait}}(t) = \{T_{\text{wait}_1}, \dots \}$ denote the set of waiting times of all started ride requests $A_i \to B_i$ up to time $t$. Then the comfort metric of the fleet management task is the maximal waiting time stored in the set $S_{wait}$.

$$
\objective_{C-FM}(t) = \max_{T_{\text{wait}}} S_{\text{wait}}
$$


This concludes the exposition of the rules of the AI Driving Olympics. Rules and their evaluation are subject to changes to ensure practicability and fairness of scoring.
