# Duckietown Machine Learning Primer {#ml-primer status=draft}

Maintainer: Florian Golemo (but somebody please have a look at this)

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

**Rewards/Penalties:** With every observation (input) you also receive a scalar real value in range [-1000,1000] corresponding to how good or bad your last action was. In the `LF` and `LFV` tasks, this value corresponds to how well you stay in the center of the right lane, how your steering angle compares to the racing line of the current tile (e.g. if it's a turn how well are you following the optimal trajectory). If you go off the road (not just into the oncoming lane, but right off the edge of the map) you get a one-time `-1000` reward, the current episode is over, and your Duckiebot gets reset to a random spot on the map. The same thing happens if you bump into some of the static (`LF`) or dynamic (`LFV`) obstacles. *Important: you only get this reward signal during training in simulation. When running your agent on the real robot, i.e. during evaluation, the reward signal is `0` at every time step.*

---

## Reinforcement Learning

Reinforcement learning in general means that your learning agent receives inputs (**"observations"**) at every timestep, and has to provide an output (**"action"**) in return, but you don't know what the correct action is. In order to guide the action, however, you receive a feedback (**"reward"**) on how good or bad kind of a state you are in now (after this last action). Just to make that entirely clear: *the reward you receive is not necessarily for the last action you took. It's the reward for the state you find yourself in now and that includes all the actions you took so far*. Therefore a mistake you made some time ago can show a lot later, or equivalently a correct move could take a few timesteps to show as higher reward.

The tasks `LF`,`LFV`, and `NAVV` follow the OpenAI Gym convention of providing reinforcement learning environments. That means each of these tasks can be used directly with existing RL algorithm implementation. Here is how it works.

The general pattern of any gym environment is this:

```python
import gym
import NAME_OF_GYM_EXTENSION

env = gym.make("NAME-OF-ENVIRONMENT")

observation = env.reset()

EPISODES = 10

done = False

for episode in range(EPISODES):
  while not done:

    action = PICK_ACTION(observation)

    observation, reward, done, misc = env.step(action)

    UPDATE_AGENT(observation, reward, done, misc)

    # this next line is optional
    env.render("human")

  env.reset()

env.close()
```

The ingredients you need to make this work are these:

- `NAME_OF_GYM_EXTENSION` - Either you are using a gym environment that ships with OpenAI gym (then you don't need this line), or you are using a custom environment like the one we are providing for this task and then you have to import the custom environment in addition to `gym`. This is `gym_duckietown_agent` in our specific case.
- `NAME-OF-ENVIRONMENT` - Is the name of the environment of the concrete reinforcement learning problem you are trying to solve, so for example in our case for every task there is one such environment. The names are `Duckietown-Lf-v0`, `Duckietown-Lfv-v0`, and `Duckietown-Navv-v0` for the tasks `LF`, `LFV`, and `NAVV` respectively.
- `EPISODES` - Is a number (integer `≥1` ) of your choosing. During training this will determine how long your algorithm is training. During testing this will determine how well your reward is averaged. The latter means that between individual episodes you can get wildly different results due to randomness. Therefore you should evaluate with a high number for more consistent results. Each episode will automatically run only for a given number of timesteps. This is determined by each task and you can look it up in the source code if you want.
- `PICK_ACTION` - This is your agent code for selecting an action, i.e. the inference part. You have to implement this by yourself. The selection of an action is usually dependent on the last known observation, but feel free to do this however you like. You can also for example completely ignore the last observation and just randomly sample a possible action from the action space like so: `action = env.action_space.sample()`. That's a function that all gym environments include.
- `UPDATE_AGENT` - This is also your agent code, but this time it's the function that adds new knowledge to your agent. When you're implementing this function, you usually pay attention to the observation you got with respect to the previous observation and your action (i.e. to learn how your action affects the environment), to the reward you got w.r.t to the action (i.e. to learn how your action affects rewards), or to the misc variable to see if the environment has any additional data for you, or any combination of the above. `done` is usually also included in learning agents to see which actions lead to a termination of the episode. This can be either catastrophic failure like driving into obstacles or this can be "good" and just your agent reaching the maximum possible steps for an episode.

As for what the standard gym functions are doing:

- `env.reset()` - Is used (a) to initialize the environment and (b) to reset the environment in case of failure or max steps. This function always returns an observation.
- `env.step(action)` - Takes an action and executes it in the environment. This returns a 4-tuple of next observation, reward, done-ness, and misc. Done-ness means is the current episode over? If this is `True`, then the environment needs to be `reset()`.
- `env.render(mode)` - This is entirely optional. Many environments support rendering the state of the environment so that researchers can observe their agents (either during training or testing). In order to get the human observer mode, run this with `mode = "human"`. Another common example is `env.render("rgb_array")` which returns the same observation, but not as a graphic or window but as a NumPy array. In our case the observation in the `LF`, `LFV`, and `NAVV` tasks is the camera feed and the `env.render()` shows you the camera feed with some additional data on top that the agent doesn't see (like the center of the road, orientation, etc.).
- `env.close()` - Is mostly here to adhere to the convention. There are some environments that need to be explicitly closed. Our doesn't. But it doesn't hurt to keep a good coding style and conform to this simple convention.

If you want to see know about reinforcement learning or see an RL algorithm in action, please check out: (TODO put link to next page here, 20_reinforcement_learning.md).

---

## Supervised Learning (Imitation Learning)

TODO

(TODO put link to next page here, 30_imitation_learning.md).

---

## Transfer Learning

TODO how do you transfer what you learned through imitation learning into a RL algorithm?
