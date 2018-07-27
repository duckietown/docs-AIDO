# Duckietown Machine Learning Primer {#ml-primer status=draft}

In here we'll show you what kinds of machine learning are possible in the context of our challenge and what they look like in practice.

## Overview

This diagram should give you an idea of what kind of machine learning is applicable to each challenge.

<figure id='ml-overview'>
<figcaption></figcaption>
<img src="images/ml-overview.png" class='diagram' style="width:100%"/>
</figure>

---

Please take this with a grain of salt - these are just recommendations. Feel free to surprise us with cool or novel combinations of methods.

## Input and Output Data

#### LF, LFV, NAVV challenges

For all these challenges the task is to control a single Duckiebot by sending it the motor commands that solve the respective task.

**Inputs (what your machine learning agent receives):** One image per timestep as NumPy `ndarray`, resolution `160x120x3` in range [0,255] with RGB encoding. In simulation, every time you receive the next image exactly 1/30th of a second of simulation time has passed. As long as you don't act, the simulation is frozen. So you can grab an image, process it for 5 minutes, return a motor command, and then grab the next image which is still only 1/30s later in simulation time. However on the real robot time obviously doesn't stand still. The images are dealt out at roughly 30Hz, but you don't have to grab them at the same speed - you'll always get the latest image. For example, if your agent code usually takes around 0.5s to process a single image and generate the best motor command, then the next image you grab will be the one from ~0.5s after the previous one. Therefore it's very important that you test your algorithm for speed and homogeneous inference time. If your inference time varies too strongly you'll very likely run into problems controlling the real robot reliably. *Best practice: optimize your inference speed so that you can make an inference at least every 1/30s.*

**Inputs (`NAVV` only):** For the `NAVV`task you receive some additional data (i.e. in addition to the camera image)... map, starting position and orientation (`A`) and goal position (`B`)... TODO, what representation, how often (only at the beginning vs. at every time step)

**Outputs (what your machine learning agent has to produce/learn):**
One pair of motor commands per timestep in the form of `(v,omega)`, where `v` = continuous velocity in range [-1,1] corresponding to maximum power backwards (-1) and maximum power forward (+1) and `omega` = continuous steering angle in range [-1,1] corresponding to hard left steering (-1) and hard right steering (+1). The robot has a minimum turn radius that is automatically taken into account, i.e. if you try to do something like `(0,1)` or `(0,-1)` the robot will *not* rotate in-place.

**Rewards/Penalties:** With every observation (input) you also receive a scalar real value in range [-1000,1000] corresponding to how good or bad your last action was. In the `LF` and `LFV` tasks, this value corresponds to how well you stay in the center of the right lane, how your steering angle compares to the racing line of the current tile (e.g. if it's a turn how well are you following the optimal trajectory). If you go off the road (not just into the oncoming lane, but right off the edge of the map) you get a one-time `-1000` reward and your Duckiebot gets reset to a random spot on the map. The same thing happens if you bump into some of the static (`LF`) or dynamic (`LFV`) obstacles.
