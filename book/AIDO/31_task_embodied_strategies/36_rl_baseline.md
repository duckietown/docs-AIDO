# Reinforcement Learning {#embodied_rl status=beta}

This section describes the basic procedure for making a submission with a model trained in simulation using reinforcement learning with PyTorch. It can be used as a starting point for any of the [`LF`](#lf), [`LFV`](#lf_v), and [`LFVI`](#lf_v_i) challenges.

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [PyTorch template](#pytorch-template).

Result: You win the AI-DO!

</div>


## Quickstart

To train a policy:
    
1) Clone [this repo](https://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch)

    $ git clone git://github.com/duckietown/challenge-aido_LF-baseline-RL-sim-pytorch.git
    
2) Change into the directory:
    
    $ cd challenge-aido_LF-baseline-RL-sim-pytorch
        
3) Install this package

    $ pip3 install -e . # if you are in a conda env
    $ sudo pip3 install -e .  # if you want to install this system-wide

and install gym-duckietown (Use `sudo` if system-wide)

    $ pip3 install -e git://github.com/duckietown/gym-duckietown.git#egg=gym-duckietown
        
(4) Run the training script

    $ python3 2-train-ddpg-cnn.py --seed 123
        
(5) When it finishes, check it out (but first edit this following file and set the seed to the one you used above, like `123` in line 10)

    $ python3 3-test-ddpg-cnn.py
        
## How to submit the trained policy

Once you're done training, you need to copy your model and the saved weights of the policy network.

Specifically if you use this repo then you need to copy the following artifacts:

- `duckietown_rl/ddpg.py` and rename to `model.py`
- `scripts/pytorch_models/DDPG_2_XXX_actor.pth` and `..._XXX_critic.pth` and rename to `models/model_actor.pth` and `models/model_critic.pth` respectively, where `XXX` is the seed of your best policy
- `duckietown_rl/wrappers.py` to just `wrappers.py` and don't rename. :)

Then edit the `solution.py` file over to make sure everything is loaded correctly (i.e. all of the imports point to the right place) and execute

    $ dts challenges submit 
    
to send your solution.

## How to improve your policy

Here are some ideas for improving your policy:

- Check out the `DtRewardWrapper` and modify the rewards (set them higher or lower and see what happens)
- Try making the observation image grayscale instead of color. And while you're at it, try stacking multiple images, like 4 monochrome images instead of 1 color image
- You can also try increasing the contrast in the input. to make the difference between road and road-signs clearer. You can do so by adding another observation wrapper.
- Cut off the horizon from the image (and correspondingly change the convnet parameters). 
- Check out the default hyperparameters in `duckietown_rl/args.py` and fiddle around with them; see if they work better. For example increase the `expl_noise` or increase the `start_timesteps` to get better exploration.
- (more sophisticated) Use a different map in the simulator, or - even better - use randomized maps.
- (more sophisticated) Use a different/bigger convnet for your actor/critic. And add better initialization.
- (super sophistacted) Use the ground truth from the simulator to construct a better reward  
- (crazy sophisticated) Use an entirely different training algorithm - like PPO, A2C, or DQN. Go nuts. But this might take significant time, even if you're familiar with the matter.

## Optional: track experiment progress with HyperDash

Install Hyperdash (an open source experiment tracker and plotting tool), and create a profile or login (for more info check out [hyperdash.io](https://hyperdash.io)

    pip install hyperdash && hyperdash signup # for conda users
    # OR for system-wide installation
    sudo pip install hyperdash && hyperdash signup

Install the HyperDash app for Android/iOS. You can follow the progress of he various experiments there and you'll get a push notification when your experiment finishes or breaks for some reason.
