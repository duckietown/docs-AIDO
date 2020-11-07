# Using the Simulator {#aido-simulator status=beta}


Doing great on the simulated challenges, but not on the real evaluation? 
Or doing great in your training, but not on our simulated, held-out environments? 

In several of the baselines (namely [IL from sim](#embodied_il_sim), [RL](#embodied_rl), [Classical Duckietown](#ros-baseline) part of the workflow for improving your submission includes interacting with the simulator locally. For example, in [IL from sim](#embodied_il_sim), you use the simulator to generate examples to imitate, in [RL](#embodied_rl) the simulator gives you the reward, and in [Classical Duckietown](#ros-baseline) you can run your submission locally for faster development. 

In all of these cases, you can modify the parameters of the simultor. In each case, you will see a file called `env.py`. In this file,  we launch the `Simulator` class from `gym-duckietown`:

```
        from gym_duckietown.simulator import Simulator
        env = Simulator(
            seed=123, # random seed
            map_name="loop_empty",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4, # start close to straight
            full_transparency=True,
            distortion=True,
        )
```

When we [take a look at the constructor](https://github.com/duckietown/gym-duckietown/blob/aido2_lf_r1/gym_duckietown/simulator.py#L145-L180), you'll notice that we aren't using all of the parameters listed. In particular, the three you should focus on are:
    
- `map_name`: What map to use; hint, take a look at gym_duckietown/maps for more choices
- `domain_rand`: Applies domain randomization, a popular, black-box, sim2real technique
- `randomized_maps_on_reset`: Slows training time, but increases training variety.
- `camera_rand`: Randomizes the camera calibration to increase variety.
- `dynamics_rand`: Simulates a miscalibrated Duckiebot, to better represent reality.


Mixing and matching different values for these will help you improve your training diversity, and thereby improving your evaluation robustness!

If you're interested in more advanced techniques, like learning a representation that is a bit easier for your network to work with, or one that transfers better across the simulation-to-reality gap, there are some [alternative, more advanced methods](https://github.com/duckietown/segmentation-transfer) you may be interested in trying out.
