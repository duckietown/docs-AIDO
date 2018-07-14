# Task: Lane following + Dynamic vehicles (LFV) {#lf_v status=draft}

The second task of the *AI Driving Olympics* is "Lane following with dynamic vehicles".
This task is an extension of Task LF to include additional rules of the road and other moving vehicles and static obstacles.

Again we ask participants to submit code allowing the Duckiebot to drive on the right-hand side of the street within Duckietown. Due to interactions with other Duckiebots, the task is designed no longer completely \emph{reactive}. Intersections will be recognized and maneuvered using provided code from the organizers. 


<div figure-id="fig:others1">
<img src='images/lane_following_v.jpg' figure-id="subfig:lane_following_v" style="width:90%"/>
<img src='images/in_lane_sideview.jpg' figure-id="subfig:lane_following_v_ego"  style="width:90%"/>
</div>

<div figure-id="fig:others2">
<img src='images/collision1.jpg' figure-id="subfig:collision1"  style="width:90%"/>
<img src='images/yield.jpg' figure-id="subfig:aido-yield"  style="width:90%"/>
</div>



The robot used in this task is a Duckiebot as described in [](#robot). The environment of the task is Duckietown as described in [](#environment). The main addition to the lane following task is that now obstacles may appear on the road which shall be avoided.



## Evaluation

The lane following task is evaluated on three separate objectives.

### Performance objective

The [*performance objective*](#performance_lf) measures how fast a Duckiebot moves.


### Traffic law objective

The following traffic laws apply in the lane following with dynamic vehicles task.

* [Staying in the lane](#traffic_laws_lf)
* [Collision avoidance](#traffic_laws_ac)



### Comfort objective

The following objective quantifies how "comfortable" a Duckiebot is driving.

[Comfortable driving](#comfort_embodied)
