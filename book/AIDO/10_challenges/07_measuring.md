# Performance metrics {#measuring-performance status=ready}

Measuring performance in robotics is less clear cut and more multidimensional than traditionally encountered in machine learning settings. Nonetheless, to achieve reliable performance estimates we assess submitted code on several *episodes* with different initial settings and compute statistics on the outcomes. We denote $\objective$ to be an objective or cost function to optimize, which we evaluate for every experiment. In the following formalization, objectives are assumed to be minimized.

In the following we summarize the objectives used to quantify how well an embodied task is completed. We will produce scores in three different categories:

## Performance criteria (P) {#performance status=ready}

### Lane following (LF / LFV) {#performance_lf}

As a performance indicator for both the "lane following task" and the "lane following task with other dynamic vehicles", we choose the integrated speed $v(t)$ along the road (not perpendicular to it) over time of the Duckiebot. This measures the moved distance along the road per episode, where we fix the time length of an episode. This encourages both faster driving as well as algorithms with lower latency. An *episode* is used to mean running the code from a particular initial configuration.


$$
\objective_{P-LF(V)}(t) = \int_{0}^{t} - v(t) dt
$$

The integral of speed is defined over the traveled distance of an episode up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode.

The way we measure this is in units of "tiles traveled":

$$
\objective_{P-LF(V)}(t) = \text{# of tiles traveled}
$$


<!-- ## Navigation (NAVV) {#performance_navv}
Similarly, for the "navigation with dynamic vehicles task" (NAVV), we choose the time it takes to go from point $A$ to point $B$ within a Duckietown map as performance indicator. A trip from $A$ to $B$ is *active* as soon as it is received as long as it has not been completed.


This is formalized in the equation and integral below.
$$
\objective_{P-NAVV}(t)  =  \int_{0}^{t}  \mathbb{I}_{AB-active} dt
$$

The indicator function $\mathbb{I}_{AB-active}$ is $1$ if a trip is *active* and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode. -->

<!-- ## Fleet management (FM) {#performance_fm}

As performance objective on task FM, we calculate the sum of trip times to go from $A_{i}$ to $B_{i}$. This generalizes the objective from task NAVV to multiple trips. The difference to task NAVV is that now multiple trips $(A_{i},B_{i})$ may be active at the same time. A trip is *active* as soon as it is requested and as long as it has not been completed. Likewise, multiple Duckiebots are now available to service the additional requests. To reliably evaluate the metric, multiple pairs of points A, B will be sampled at different time points within an episode.

$$
\objective_{P-FM}(t) =  \sum_i \int_{0}^{t} \mathbb{I}_{i-active} dt
$$

The indicator function $\mathbb{I}_{i-active}$ is $1$ if a trip is \emph{active} and $0$ otherwise. Again the integral of an episode is defined up to time $t=T_{eps}$, where $T_{eps}$ is the length of an episode. -->




## Traffic law objective (T) {#traffic_laws status=ready}

The following shows rule objectives the Duckiebots are supposed to abide by within Duckietown. These penalties hold for the embodied tasks (LF, LFV).
<!-- All individual rule violations will be summarized in one overall traffic law objective $\objective_{T}$.  -->

### Major infractions

This objective means to penalize "illegal" driving behavior. As a cover for many undesired behaviors, we count the median time spent oustide of the drivable zones. This also covers the example of driving in the wrong lane.

Metric: The median of the time spent outside of the drivable zones.

\begin{align*}
\mathcal{J}_{T-LF/LFV} = \text{median}(\{t_{outside}\}),
\end{align*}

where $\{t_{outside}\}$ is the list of accumulated time outside of drivable zones per episode. 

<!-- ### Quantification of "Staying in the lane" {#traffic_laws_lf}
TODO: To be implemented

<div figure-id="fig:crossing_lane">
<img src="images/crossing_lane.jpg" style="width: 80%"/>
<figcaption>Picture depicting a situation in which the "staying-in-the-lane rule" applies.</figcaption>
</div>

The Duckietown traffic laws say:

"The vehicle must stay at all times in the right lane, and ideally near the center of the right lane."

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

An example situation where a Duckiebot does not stay in the lane is shown in \ref{fig:crossing_lane}. -->


<!-- ## Intersection navigation {#traffic_intersection}

Traditionally for real-world car travel, intersections are traversed with a fixed protocol in place, e.g. in Germany the driver on the right joining lane has the right of way.

Robotic drivers do not have these inherent restrictions. Therefore for the task of [navigation](#nav_v), we will provide a base intersection protocol. This protocol is able to guide Duckiebots safely through intersections. For the lane following tasks, a fixed intersection protocol will be used and the performance during intersection navigation will not be scored.

Participants are then able to change the intersection protocol in the hope of improving driving through intersections, thereby enabling them to achieve better navigation. The following rule penalties around intersections will however stay in place. -->


<!-- ### Quantification of "Stopping at red intersection line" and "Stopping at red traffic light" {#traffic_laws_si}
TODO: To be implemented or removed

There are two different possibilities forcing the Duckiebot to a stop at an intersection. Some intersections have red stopping lines whereas others have traffic lights. The stopping behavior in both cases is similar and serves a similar purpose however. We therefore join the two cases into the "stopping at intersection"-rule.

 The Duckietown traffic laws say:

 "Every time the vehicle arrives at an intersection with a red stop line,
 the vehicle should come to a complete stop  in front of it, before continuing."

<div figure-id="fig:intersection">
<img src="images/intersection_stop.jpg" style="width:80%"/>
<figcaption>Picture depicting a Duckiebot stopping at a red intersection line.
</figcaption>
</div>

Likewise, the traffic law says that:
"Every time the vehicle arrives at an intersection with a red traffic light,
the vehicle should come to a complete stop in front of it, and shall remain at rest as long as the red light is turned on."

During each intersection traversal, the vehicle is penalized by $\gamma$ if either of the above stopping rules are violated.

Let $\mathbb{I}_{SI1}$ denote the red intersection line stopping rule as an indicator function.

The red line stopping rule applies if there was not a time $t$ when the vehicle was at rest ($v(t) = 0$) in the stopping zone defined as the rectangular area of the same width as the red line between $a$ and $b$ cm distance from the start of the stop line perpendicular to the center of mass point of the Duckiebot. This situation is demonstrated in Fig. \ref{fig:intersection}. $a$ and $b$ will be determined empirically to ensure reasonable behavior.

$$
\mathbb{I}_{SI1} = \begin{cases} 1, \quad {\nexists t \text{ s.t. } v(t)=0 \wedge  p(t) \in S_{zone}} \\
0,\quad otherwise
\end{cases}
$$

The condition that the position $p(t)$ of the center of mass of the Duckiebot
is in the stopping zone is denoted with $p(t) \in \mathcal{S}$.

Let $I_{SI2}$ denote the red traffic light stopping rule.

$$
\mathbb{I}_{SI2} = \begin{cases} 1, \quad {p(t) \text{ crosses intersection} \wedge \text{ traffic light red}} \\
0,\quad \text{ otherwise}
\end{cases}
$$

Then we write the objective as the cumulative sum of stopping at intersection rule infractions. The sum is over all intersection time periods, in which a rule violation may have occurred.

 $$
      \objective_{T-SI}(t) = \sum_{t_k} \gamma  (\mathbb{I}_{SI1} + \mathbb{I}_{SI2})
 $$

Here the sum over time increments $t_k$ denote the time intervals in which this conditions is checked. The rule penalty is only applied once the Duckiebot leaves the stopping zone. Only then is it clear that it did not stop within the stopping zone.

To measure this cost, the velocities $v(t)$ are evaluated while the robot is in the stopping zone $\mathcal{S}$. An example of a Duckiebot stopping at a red intersection line is depicted in Fig. \ref{fig:intersection}.

### Quantification of "Keep safety distance" {#traffic_laws_sd}
TODO: To be implemented

The Duckietown traffic laws say:

 "Each Duckiebot should stay at an adequate distance from the Duckiebot in front of it, on the same lane, at all times."

We quantify this rule as follows: Let $b(t)$
denote the distance between
the center of mass of the Duckiebot and the center of mass of the closest Duckiebot in front of it which is also in the same lane. Furthermore let $b_{\text{safe}}$ denote a cut-off distance after which a Duckiebot is deemed "far away". Let $\delta$ denote a scalar positive weighting factor. Then

$$
\objective_{T-SD}(t) = \int_0^t \delta \cdot \max(0,b(t)- b_{\text{safe}})^2.
$$ -->

<!-- ### Quantification of "Avoiding collisions" {#traffic_laws_ac}
TODO: To be implemented

The Duckietown traffic laws say:

*At any time a Duckiebot shall not collide with a duckie, Duckiebot or object.*

<div figure-id="fig:collision">
<img src="images/collision1.jpg" style="width:80%"/>
<figcaption>Picture depicting a collision situation.
</figcaption>
</div>

Collisions in Duckietown are generally not desired. -->
<!-- We distinguish additionally what the controlled Duckiebot collides with. For collisions with people of Duckietown (duckies) a higher penalty $\nu_1$ is incurred as for collisions with other cars $\nu_2)$ (Duckiebots) or objects $\nu_3$. -->

<!-- The vehicle is penalized by $\nu$ if within a time a time interval of length $t_k$ $t \in [t, t+t_k)$, the distance $\ell(t)$ between the vehicle and a nearby duckie, object or other vehicle is zero or near zero. $\ell(t)$ denotes the perpendicular distance between any object and the Duckiebot rectangular surface. The collision cost objective therefore is

  \begin{align*}
     \objective_{T-AC}(t) = \sum_{t_k} \nu \mathbb{I}_{\exists t \in [ t-t_k, t ) \ell(t) < \epsilon}
  \end{align*}

where $\nu$ is the penalty constant of the collision. -->

<!-- $$ -->
<!-- \text{Duckie collision } \nu_1 > \text{Duckiebot collision } \nu_2 > \text{Object collision } \nu_3 -->
<!-- $$ -->

<!-- Time intervals are chosen to allow for maneuvering after collisions without incurring further costs.

An illustration of a collision is displayed in Fig. \ref{fig:collision}. -->

<!-- #### Quantification of "Yielding the right of way" {#traffic_laws_yr}

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

The yield situation at an intersection is depicted in Fig.~\ref{fig:yield}. -->

<!-- ### Hierarchy of rules {#traffic_laws_hierarchy}
<!-- TODO: finalize this section -->

<!-- To account for the relative importance of rules, the factors $\alpha, \beta, \gamma, \delta, \nu$ of the introduced rules will be weighted relatively to each other.

Letting $>$ here denote "more important than", we define the following rule hierarchy:

$$
\objective_{T-AC} > \objective_{T-SI} > \objective_{T-SD} > \objective_{T-LF}
$$

I.e.:

\begin{center}Collision avoidance $>$ Stop line $>$ Safety distance $>$ Staying in the lane.
\end{center}

This constrains the factors $\alpha, \beta, \gamma, \delta, \nu$ whose exact values will be determined empirically to enforce this relative importance.

While the infractions of individual rules will be reported, as a performance indicator all rule violations are merged into one overall traffic law objective $\objective_{T}$. Let $\task$ denote a particular task, then the rule violation objective is the sum of all individual rule violations $\objective_i$ which are an element of that particular task.

$$
\objective_{T} = \sum_i \mathbb{I}_{\objective_i \in \task} \objective_{T-i},
$$

where $\mathbb{I}_{\objective_i \in \task}$ is the indicator function that is $1$ if a rule belongs to the task and $0$ otherwise. -->


## Comfort objective (C) {#comfort status=ready}

### Lane following (LF, LFV) {#comfort_embodied}

In the single robot setting, we encourage "comfortable" driving solutions. We therefore penalize large angular deviations from the forward lane direction to achieve smoother driving. This is quantified through changes in Duckiebot angular orientation $\theta_{bot}(t)$ with respect to the lane driving direction.
<!-- Smoothing is performed by convolving the Duckiebot position $p_{bot}(t)$ with a smoothing filter $k_{smooth}$. -->

#### Lateral deviation

For better driving behavior we measure the median per episode lateral deviation from the right lane center line.

$$
\objective_{C-LF/LFV}(t) = \text{median}(\{d_{outside}\}),
$$

where $\{d_{outside}\}$ is the sequence of lateral distances from the center line.

<!-- ## Fleet management (FM) {#comfort_fm}

In the fleet management setting "customer experience" is influenced greatly by how fast and dependable a service is. If it is known that a taxi arrives quickly after ordering it, it makes the overall taxi service more convenient.

We therefore define the comfort objective as the maximal waiting time $T_{wait}$ until customer pickup. Let $T_{wait}$ denote the time beginning at the reception of a ride request until when the ride is started.


Let $S_{\text{wait}}(t) = \{T_{\text{wait}_1}, \dots \}$ denote the set of waiting times of all started ride requests $A_i \to B_i$ up to time $t$. Then the comfort objective of the fleet management task is the maximal waiting time stored in the set $S_{wait}$.

$$
\objective_{C-FM}(t) = \max_{T_{\text{wait}}} S_{\text{wait}}
$$ -->
