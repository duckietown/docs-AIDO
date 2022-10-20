<!-- # Task: Navigation + Dynamic vehicles (NAVV) {#nav_v status=beta}

The third task of the *AI Driving Olympics* is "Navigation with dynamic vehicles".
This task is an extension of task LF and task LFV and now focuses on navigating from location "A" to location "B" within Duckietown. The task also includes a map of Duckietown as input.

Again we ask participants to submit code allowing the Duckiebot to navigate from location to location by driving on the right-hand side of the street within Duckietown. Given interactions with other Duckiebots, the additional rules of the road compared to the lane following tasks have to be respected.

An important addition to this task is the possibility to change the [intersection navigation protocol](#traffic_intersection).

<img src="images/fleet_management.jpg" figure-id="fig:fleet_management" figure-caption="Map of a Duckietown provided to illustrate Task NAVV and FM."  style="width:90%"/>


### Performance objective

As performance objective on the task, we denote the expected time to go from point A to point B. To reliably evaluate the metric, multiple pairs of points A, B will be sampled.


The robot used in this task is a Duckiebot as described in [](#robot). The environment of the task is Duckietown as described in [](#environment). Different to task [LF](#challenge-LF) and [LFV](#challenge-LF_v), the input to the Duckiebot now also includes a map of Duckietown.

## Evaluation

The navigation with dynamic vehicles task is evaluated on three separate objectives. A crucial addition compared to lane following is that now also intersections need to be traversed successfully.

### Performance objective

The [*performance objective*](#performance_navv) measures how fast a Duckiebot can navigate from starting points to end points within Duckietown.


### Traffic law objective

The following traffic laws apply in the navigation task.

* [Staying in the lane](#traffic_laws_lf)
* [Collision avoidance](#traffic_laws_ac)
* [Stopping at intersections](#traffic_laws_si)
* [Keeping a safety-distance](#traffic_laws_sd)


### Comfort objective

The following objective quantifies how "comfortable" a Duckiebot is driving.

[Comfortable driving](#comfort_embodied) -->
