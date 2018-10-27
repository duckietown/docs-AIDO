# Pytorch template {#pytorch-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [PyTorch](https://pytorch.org/).

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission and see your entry on the [Leaderboard](https://challenges.duckietown.org/).

</div>

## Quickstart

1. Clone this repo:

        git clone https://github.com/duckietown/challenge-aido1_LF1-template-pytorch.git

2. Change into the directory that you cloned:
    
        cd challenge-aido1_LF1-template-pytorch
        
3. Submit :)

        dts challenges submit
        
4. Once this finishes, you'll receive a link where you can follow the progress of your evaluation.


## Description

This is a simple template for an agent that uses PyTorch/DDPG for inference.

[This code is documented here](https://docs.duckietown.org/DT18/AIDO/out/pytorch_template.html).

## How to change the model

Once you trained your own policy, check out line 20-45 of file [solution.py](solution.py#L20).

Basically, you need to copy your PyTorch network here (and probably replace the `model.py` file with one that contains your own model). And also copy the weights of the network you trained into the `models` directory.

Also include any gym wrappers if you included any during training (however, only observation/action wrappers, not reward wrappers).

Finally, change the code in `solution.py` line 20-45 to represent the steps to load your model and load the saved network weights and the wrappers.
