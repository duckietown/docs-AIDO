# Reinforcement Learning {#embodied_rl status=ready}

This section describes the basic procedure for making a submission with a model trained in simulation using reinforcement learning with PyTorch. 

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [PyTorch template](#pytorch-template).

Result: Your expectations in regards to RL's capabilities are tempered

</div>

Before getting started, you should be aware that RL is very much an active area of research. Simply getting a successful turn with
this baseline should be celebrated. It is still provided to you because this implementation is a good stepping point to other algorithms, but
do not expect this to converge to an AIDO-winning policy out of the box.

You should also make sure you have access to good hardware. A recent graphics card (probably GTX1060+) is a must, and more than 8GB of RAM is required.


## Quickstart

You should install CUDA10.2+ locally. This baseline works with [CUDA 11](https://developer.nvidia.com/cuda-downloads), and it
should also work with CUDA 10.2.

To train a policy:
    
1) Clone [this repo](https://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch)
    
        $ git clone -b daffy git://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch.git

2) Change into the directory:
    
        $ cd challenge-aido_LF-baseline-RL-sim-pytorch
        
3) Install this package

        $ pip3 install -e . # Depending on your installation, might be pip instead of pip3
    and install gym-duckietown

        $ pip3 install -e git://github.com/duckietown/gym-duckietown.git@daffy#egg=gym-duckietown
        
4) Change into the `duckietown_rl` directory and run the training script

        $ cd duckietown_rl
        $ python3 -m scripts.train_cnn.py --seed 123
        
5) When it finishes, check it out (make sure you pass in the same seed as the one passed to the training script)

        $ python3 -m scripts.test_cnn.py --seed 123
        
## How to submit the trained policy

Once you're done training, you need to copy your model and the saved weights of the policy network.

Specifically if you use this repo then you need to copy the following artifacts into the corresponding locations of the root directory:

- `duckietown_rl/ddpg.py` and rename to `model.py`
- `scripts/pytorch_models/DDPG_XXX_actor.pth` and `DDPG_XXX_critic.pth` and rename to `models/model_actor.pth` and `models/model_critic.pth` respectively, where `XXX` is the seed of your best policy

Also, make sure that the root-level `wrappers.py` contains all the wrappers you used in `duckietown_rl/wrappers.py`.

Then edit the `solution.py` file over to make sure everything is loaded correctly (i.e. all of the imports point to the right place) and execute

    $ dts challenges submit 
    
to send your solution.

## How to improve your policy

Here are some ideas for improving your policy:

- Check out the `DtRewardWrapper` and modify the rewards (set them higher or lower and see what happens)
- Try resizing the images. Make them smaller to have faster training, or bigger for making sure that RL can extract everything it can from them. You will need to also edit the layers in `ddpg.py` accordingly.
- Try making the observation image grayscale instead of color. And while you're at it, try stacking multiple images, like 4 monochrome images instead of 1 color image. You will need to also edit the layers in `ddpg.py` accordingly.
- You can also try increasing the contrast in the input to make the difference between road and road-signs clearer. You can do so by adding another observation wrapper.
- Cut off the horizon from the image (and correspondingly change the convnet parameters). 
- Check out the default hyperparameters in `duckietown_rl/args.py` and fiddle around with them; see if they work better. For example increase the `expl_noise` or increase the `start_timesteps` to get better exploration.
- (more sophisticated) Use a different map in the simulator, or - even better - use randomized maps. But be mindful that some maps include obstacles on the road, which might be counter-productive to a `LF` submission.
- (more sophisticated) Use a different/bigger convnet for your actor/critic. And add better initialization.
- (super sophisticated) Use the ground truth from the simulator to construct a better reward  
- (crazy sophisticated) Use an entirely different training algorithm - like PPO, A2C, or DQN. Go nuts. But this might take significant time, even if you're familiar with the matter.

## Sim2Real Transfer (Optional)

Doing great on the simulated challenges, but not on the real evaluation? Or doing great in your training, but not on our simulated, held-out environments? Take a look at `env.py`. You'll notice that we launch the `Simulator` class from `gym-duckietown`. When we [take a look at the constructor](https://github.com/duckietown/gym-duckietown/blob/daffy/src/gym_duckietown/simulator.py), you'll notice that we aren't using all of the parameters listed. In particular, the three you should focus on are:
    
- `map_name`: What map to use; hint, take a look at gym_duckietown/maps for more choices
- `domain_rand`: Applies domain randomization, a popular, black-box, sim2real technique
- `randomized_maps_on_reset`: Slows training time, but increases training variety.

Mixing and matching different values for these will help you improve your training diversity, and thereby improving your evaluation robustness!

If you're interested in more advanced techniques, like learning a representation that is a bit easier for your network to work with, or one that transfers better across the simulation-to-reality gap, there are some [alternative, more advanced methods](https://github.com/duckietown/segmentation-transfer) you may be interested in trying out. In addition, don't forget to try using the [logs infrastructure](http://logs.duckietown.org/), which you can also use to do things like [imitation learning](https://github.com/duckietown/challenge-aido_LF-baseline-IL-logs-tensorflow/)!

## Training headless

Should you want to train on a server, you will notice that the simulator requires an X server to run. Fear not, however, as we can 
use a fake X server for it. 

    $ xvfb-run -s "-screen 0 1400x900x24" python3 -m scripts.train_cnn.py --seed 123
    
That way, we trick the simulator into thinking that an X server is running. And, to be honest, from its point of view, it's actually true!

## Controlling which GPU is being used

Your machine might have more than one GPU. To select the nth instead of the 0th, you can use

    $ CUDA_VISIBLE_DEVICES=n python3 -m scripts.train_cnn.py --seed 123
    
This is, of course, combinable with running on a server

    $ CUDA_VISIBLE_DEVICES=n xvfb-run -s "-screen 0 1400x900x24" python3 -m scripts.train_cnn.py --seed 123