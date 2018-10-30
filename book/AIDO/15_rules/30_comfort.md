# Comfort objective {#comfort status=ready}

## Lane following and navigation (LF, LFV, NAVV) {#comfort_embodied}
TODO: To be implemented

In the single robot setting, we encourage "comfortable" driving solutions. We therefore penalize large accelerations to achieve smoother driving. This is quantified through smoothed changes in Duckiebot position $p_{bot}(t)$. Smoothing is performed by convolving the Duckiebot position $p_{bot}(t)$ with a smoothing filter $k_{smooth}$.

As a comfort objective, we measure the smoothed absolute changes in position $\Delta p_{bot}(t)$ over time.

$$
\objective_{C-LF/LFV/NAVV}(t) = \int_0^t k_{smooth} * \Delta p_{bot}(t) dt
$$


<!-- ## Fleet management (FM) {#comfort_fm}

In the fleet management setting "customer experience" is influenced greatly by how fast and dependable a service is. If it is known that a taxi arrives quickly after ordering it, it makes the overall taxi service more convenient.

We therefore define the comfort objective as the maximal waiting time $T_{wait}$ until customer pickup. Let $T_{wait}$ denote the time beginning at the reception of a ride request until when the ride is started.


Let $S_{\text{wait}}(t) = \{T_{\text{wait}_1}, \dots \}$ denote the set of waiting times of all started ride requests $A_i \to B_i$ up to time $t$. Then the comfort objective of the fleet management task is the maximal waiting time stored in the set $S_{wait}$.

$$
\objective_{C-FM}(t) = \max_{T_{\text{wait}}} S_{\text{wait}}
$$ -->
