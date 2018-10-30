# Traffic law objective {#traffic_laws status=ready}

The following are a list of rule objectives the Duckiebots are supposed to abide by within Duckietown. All individual rule violations will be summarized in one overall traffic law objective $\objective_{T}$. These penalties hold for the embodied tasks (LF, LFV).

## Quantification of "Staying in the lane" {#traffic_laws_lf}
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

An example situation where a Duckiebot does not stay in the lane is shown in \ref{fig:crossing_lane}.


<!-- ## Intersection navigation {#traffic_intersection}

Traditionally for real-world car travel, intersections are traversed with a fixed protocol in place, e.g. in Germany the driver on the right joining lane has the right of way.

Robotic drivers do not have these inherent restrictions. Therefore for the task of [navigation](#nav_v), we will provide a base intersection protocol. This protocol is able to guide Duckiebots safely through intersections. For the lane following tasks, a fixed intersection protocol will be used and the performance during intersection navigation will not be scored.

Participants are then able to change the intersection protocol in the hope of improving driving through intersections, thereby enabling them to achieve better navigation. The following rule penalties around intersections will however stay in place. -->


## Quantification of "Stopping at red intersection line" and "Stopping at red traffic light" {#traffic_laws_si}
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

## Quantification of "Keep safety distance" {#traffic_laws_sd}
TODO: To be implemented

The Duckietown traffic laws say:

 "Each Duckiebot should stay at an adequate distance from the Duckiebot in front of it, on the same lane, at all times."

We quantify this rule as follows: Let $b(t)$
denote the distance between
the center of mass of the Duckiebot and the center of mass of the closest Duckiebot in front of it which is also in the same lane. Furthermore let $b_{\text{safe}}$ denote a cut-off distance after which a Duckiebot is deemed "far away". Let $\delta$ denote a scalar positive weighting factor. Then

$$
\objective_{T-SD}(t) = \int_0^t \delta \cdot \max(0,b(t)- b_{\text{safe}})^2.
$$

## Quantification of "Avoiding collisions" {#traffic_laws_ac}
TODO: To be implemented

The Duckietown traffic laws say:

*At any time a Duckiebot shall not collide with a duckie, Duckiebot or object.*

<div figure-id="fig:collision">
<img src="images/collision1.jpg" style="width:80%"/>
<figcaption>Picture depicting a collision situation.
</figcaption>
</div>

Collisions in Duckietown are generally not desired. We distinguish additionally what the controlled Duckiebot collides with. For collisions with people of Duckietown (duckies) a higher penalty $\nu_1$ is incurred as for collisions with other cars $\nu_2)$ (Duckiebots) or objects $\nu_3$.

The vehicle is penalized by $\nu_j$ if within a time a time interval of length $t_k$ $t \in [t, t+t_k)$, the distance $\ell(t)$ between the vehicle and a nearby duckie, object or other vehicle is zero or near zero. $\ell(t)$ denotes the perpendicular distance between any object and the Duckiebot rectangular surface. The collision cost objective therefore is

  \begin{align*}
     \objective_{T-AC}(t) = \sum_{t_k} \nu_j \mathbb{I}_{\exists t \in [ t-t_k, t ) \ell(t) < \epsilon}
  \end{align*}

where $\nu_j$ depends on the type of collision with

$$
\text{Duckie collision } \nu_1 > \text{Duckiebot collision } \nu_2 > \text{Object collision } \nu_3
$$

Time intervals are chosen to allow for maneuvering after collisions without incurring further costs.

An illustration of a collision is displayed in Fig. \ref{fig:collision}.

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

## Hierarchy of rules {#traffic_laws_hierarchy}
TODO: finalize this section

To account for the relative importance of rules, the factors $\alpha, \beta, \gamma, \delta, \nu$ of the introduced rules will be weighted relatively to each other.

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

where $\mathbb{I}_{\objective_i \in \task}$ is the indicator function that is $1$ if a rule belongs to the task and $0$ otherwise.
