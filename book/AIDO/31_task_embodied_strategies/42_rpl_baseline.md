# Residual Policy Learning {#embodied_rpl status=ready}

This section describes the basic procedure for making a submission with a model trained in simulation using residual 
policy learning with PyTorch and ROS. 

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [ROS template](#ros-template).

Result: You have a submission that leverages both our ROS stack and ML!

</div>

Before getting started, you should be aware that this baseline is a mishmash of the RL baseline and of the ROS template. 
It is recommended that you read the doc for each of those repos, as the workflow of this one is similar to those. Here are some links:

- [RL baseline](#embodied_rl)
- [ROS template](#ros-template)
- [Classical Duckietown baseline](#ros-baseline)

You should also make sure you have access to good hardware. A recent graphics card (probably GTX1060+) is a must, and more than 8GB of RAM is required.

## Quick explanation of the workflow

Notice that we have a problem here. This baseline uses both ROS and ML. This means that we can't train locally, as ROS requires, well, ROS.
We need to train inside a Docker where both ROS and PyTorch are installed.

Luckily, the ROS template repo already provides us with a submission docker image. Our strategy here is to directly use that "agent docker" 
during training, but we'll tack on the simulator and the training architecture on top.

This could have been done using a second running docker to provide a network interface to the simulator, but this adds unnecessary overhead since we don't
actually need the added security that comes with running things separately.

So, every time we train, we build the agent docker image, and then the "trainer docker" image builds directly `FROM` the agent docker, adding the simulator on top.

The final docker then runs the simulator and the agent in parallel, allowing the agent to directly talk to the simulator, just like we do in the other ML baselines.

<img src="./images/rpl_explanation.png" style='height: auto; width: 100%; object-fit: scale-down' />

## Quickstart

To train a policy, you should first make sure that Docker on your machine can access the GPU/CUDA. You should also install CUDA10.2+ locally.

Here's a few pointers:

- [nvidia-docker](https://github.com/NVIDIA/nvidia-docker)
- [CUDA 11](https://developer.nvidia.com/cuda-downloads)
    
Then: 
1) Clone [this repo](https://github.com/duckietown/challenge-aido_LF-baseline-RPL-ros)
    
        $ git clone -b daffy https://github.com/duckietown/challenge-aido_LF-baseline-RPL-ros.git

2) Change into the directory:
    
        $ cd challenge-aido_LF-baseline-RPL-ros
        
3)  Change into the `local_dev` directory.
        
        $ cd local_dev
    and open the `args.py` file.
This is how you will control the training and testing in this repo. For now, just change the `--test` argument to `default=False` (if you want to directly test, skip to point 4). Then, we can train.

        $ make run
    As mentioned in "Quick explanation of the workflow", this will first build two subsequent docker images. This might take a while. Then, it will train an RL policy over the ROS stack inside Docker.
        
4) When it finishes, check it out. Simply change the `--test` flag back to `default=True` in `args.py`.

        $ make run
    This will launch a simulator window on your host PC for you to view how your agent performs. You should see something like this. You can use this gif to gauge how long it takes for the testing docker to start (do note that this assumes that the two required docker images have already been built!)
    
<img src="./images/rpl_test.gif" style='height: auto; width: 100%; object-fit: scale-down' />
        
## How to submit the trained policy

Make sure that `rosagent.py` uses the right weights for your RL agent. This is controlled by the `MODEL_NAME` global variable.
Then, you can do

    $ dts challenges submit 
    
to send your solution.

## How to improve your policy

First, you should probably improve the base ROS policy. By default, this baseline uses the basic `lane_following` demo that is provided in Duckietown
(the same one that is launched when you launch automatic control in `keyboard_control`).

You could build a Pure Pursuit controller, change the lane filter, etc. See the [classical Duckietown baseline](#ros-baseline) for more ideas.
To do this, you would add your new ROS packages inside of `submission_ws`.

You could also limit RL's influence over the final policy. Perhaps the current approach of giving it full control in [-1,1] action values isn't restrictive enough. Perhaps it could be better if it could only change the base policy by smaller action values.

Or perhaps it's the opposite: maybe the base policy needs to be changed by more than `1`: since the min/max value that 
the base policy can output is `1`/`-1`, the RL policy would need to be able to output from `-2` to `2` to fully correct it.

Here are some ideas for improving your policy:

- Check out the `dtRewardWrapper` in `rl_agent` and modify the rewards (set them higher or lower and see what happens). By default, this wrapper is not used: you will have to add it to `train.py`.
- Try resizing the images. Make them smaller to have faster training, or bigger for making sure that RL can extract everything it can from them. You will need to also edit the layers in `ddpg.py` accordingly.
- Try making the observation image grayscale instead of color. And while you're at it, try stacking multiple images, like 4 monochrome images instead of 1 color image. You will need to also edit the layers in `ddpg.py` accordingly.
- You can also try increasing the contrast in the input to make the difference between road and road-signs clearer. You can do so by adding another observation wrapper.
- Cut off the horizon from the image (and correspondingly change the convnet parameters). 
- Check out the default hyperparameters in `local_dev/args.py` and fiddle around with them; see if they work better. For example increase the `expl_noise` or increase the `start_timesteps` to get better exploration.
- (more sophisticated) Use a different map in the simulator, or - even better - use randomized maps. But be mindful that some maps include obstacles on the road, which might be counter-productive to a `LF` submission.
- (more sophisticated) Use a different/bigger convnet for your actor/critic. And add better initialization.
- (super sophisticated) Use the ground truth from the simulator to construct a better reward  
- (crazy sophisticated) Use an entirely different training algorithm - like PPO, A2C, or DQN. Go nuts. But this might take significant time, even if you're familiar with the matter.

## Sim2Real Transfer (Optional)

Doing great on the simulated challenges, but not on the real evaluation? Or doing great in your training, but not on our simulated, held-out environments? Take a look at `local_dev/env.py`. You'll notice that we launch the `Simulator` class from `gym-duckietown`. When we [take a look at the constructor](https://github.com/duckietown/gym-duckietown/blob/daffy/src/gym_duckietown/simulator.py), you'll notice that we aren't using all of the parameters listed. In particular, the three you should focus on are:
    
- `map_name`: What map to use; hint, take a look at gym_duckietown/maps for more choices
- `domain_rand`: Applies domain randomization, a popular, black-box, sim2real technique
- `randomized_maps_on_reset`: Slows training time, but increases training variety.

Mixing and matching different values for these will help you improve your training diversity, and thereby improving your evaluation robustness!
