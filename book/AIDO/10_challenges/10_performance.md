# Performance criteria {#performance status=ready}

## Lane following (LF / LFV) {#performance_lf}

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


## Autonomous mobility on demand (AMoD) {#performance_amod}


In an autonomous mobility-on-demand system a coordinated fleet of robotic taxis serves customers in an on-demand fashion. An operational policy for the system must optimize in three conflicting dimensions:



1. The system must perform at the **highest possible service level**, i.e., at smallest possible wait times and smallest possible journey times.
2. The system's **operation must be as efficient as possible**, i.e., it must reduce its empty mileage to a minimum.
3. The system's **capital cost must be as inexpensive as possible**, i.e, the fleet size must be reduced to a minimum.

We consider robotic taxis that can carry one customer. To compare different AMoD system operational policies, we introduce the following variables:

\begin{align*}
&d_E &= &\text{ empty distance driven by the fleet} \\
&d_C &= &\text{ occupied distance driven by the fleet} \\
&d_T = d_C + d_E &= &\text{ total distance driven by the fleet} \\
&N &= &\text{ fleet size} \\
&R &= &\text{ number of customer requests served} \\
&w_i &= &\text{ waiting time of request } i\in \{1,...,R\} \\
&W &= &\text{ total waiting time } W = \sum_{i=1}^{R} w_i
\end{align*}


The provided simulation environment is designed in the standard reinforcement framework: Rewards are issued after each simulation step. The (undiscounted) sum of all rewards is the final score. The higher the score, the better the performance.

For the AMoD-Task, there are 3 different championships (sub-tasks) which constitute separate competitions. The simulation environment computes the reward value for each category and conatenates them into a vector of length 3, which is then communicated as feedback to the learning agent. The agent can ignore but the entry of the reward vector from the category that they wish to maximize.

The three championships are as follows:

### Service Quality Championship

In the **Service Quality Championship**, the principal goal of the operator is to provide the highest possible service quality at bounded operational cost. Two negative scalar weights $\alpha_1<0$ and $\alpha_2<0$ are introduced. The performance metric to maximize is

\begin{align*}
\mathcal{J}_{P-AMoD,1} = \alpha_1 W + \alpha_2 d_E
\end{align*}

The values $\alpha_1$ and $\alpha_2$ are chosen such that the term $W$ dominantes the metric. The number of robotic taxis is fixed at some fleet size $\bar{N} \in \mathbb{N}_{>0}$.




### Efficiency Championship

In the **Efficiency Championship**, the principal goal of the operator is to perform as efficiently as possible while maintaining the best possible service level. Two negative scalar weights $\alpha_3<0$ and $\alpha_4<0$ are introduced. The performance metric to maximize is

\begin{align*}
\mathcal{J}_{P-AMoD,2} = \alpha_3 W + \alpha_4 d_E
\end{align*}

$\alpha_3$ and $\alpha_4$ are chosen such that the term $d_E$ dominantes the metric. The number of robotic taxis is fixed at some fleet size $\bar{N} \in \mathbb{N}_{>0}$.



### Fleet Size Championship


In the **Fleet Size Championship**, the goal is to reduce the fleet size as much as possible while keeping the total waiting time $W$ below a fixed level $\bar{W}>0$. The condition $W\leq\bar{W}$ is equivalent to guaranteeing average waiting times smaller than $\frac{\bar{W}}{R}$. Therefore, the performance score to maximize is

\begin{align*}
\mathcal{J}_{P-AMoD,3} =
\begin{cases} -N & \text{if }W\leq\bar{W} \\
-\infty & \text{else}
\end{cases}
\end{align*}
