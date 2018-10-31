# Comfort objective {#comfort status=ready}

## Lane following (LF, LFV) {#comfort_embodied}
TODO: To be clarified - "good angle measure"

In the single robot setting, we encourage "comfortable" driving solutions. We therefore penalize large angular deviations from the forward lane direction to achieve smoother driving. This is quantified through changes in Duckiebot angular orientation $\theta_{bot}(t)$.
<!-- Smoothing is performed by convolving the Duckiebot position $p_{bot}(t)$ with a smoothing filter $k_{smooth}$. -->

As a comfort objective, we measure the average absolute changes in angular orientation $\Delta \theta_{bot}(t)$ over time.

$$
\objective_{C-LF/LFV}(t) = \frac{1}{t} \int_0^t \Delta \theta_{bot}(t) dt
$$


<!-- ## Fleet management (FM) {#comfort_fm}

In the fleet management setting "customer experience" is influenced greatly by how fast and dependable a service is. If it is known that a taxi arrives quickly after ordering it, it makes the overall taxi service more convenient.

We therefore define the comfort objective as the maximal waiting time $T_{wait}$ until customer pickup. Let $T_{wait}$ denote the time beginning at the reception of a ride request until when the ride is started.


Let $S_{\text{wait}}(t) = \{T_{\text{wait}_1}, \dots \}$ denote the set of waiting times of all started ride requests $A_i \to B_i$ up to time $t$. Then the comfort objective of the fleet management task is the maximal waiting time stored in the set $S_{wait}$.

$$
\objective_{C-FM}(t) = \max_{T_{\text{wait}}} S_{\text{wait}}
$$ -->
