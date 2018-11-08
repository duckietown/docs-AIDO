# Pytorch template for `aido1_LF1` {#pytorch-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using [PyTorch](https://pytorch.org/).

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission and see your entry on [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).

</div>

## Quickstart

### Clone the repo:

    $ git clone git://github.com/duckietown/challenge-aido1_LF1-template-pytorch.git

### Change into the directory that you cloned:
    
    $ cd challenge-aido1_LF1-template-pytorch
        
### Submit :)

    $ dts challenges submit
        
### Verify the submission

You should be able to see your submission [here](https://challenges.duckietown.org/v3/humans/challenges/aido1_LF1_r3-v3).


## Description

This is a simple template for an agent that uses PyTorch/DDPG for inference.

[This code is documented here](https://docs.duckietown.org/DT18/AIDO/out/pytorch_template.html).

## How to change the model

Once you trained your own policy, check out line 20-45 of file [solution.py](solution.py#L20).

Basically, you need to copy your PyTorch network here (and probably replace the `model.py` file with one that contains your own model). And also copy the weights of the network you trained into the `models` directory.

Also include any gym wrappers if you included any during training (however, only observation/action wrappers, not reward wrappers).

Finally, change the code in `solution.py` line 20-45 to represent the steps to load your model and load the saved network weights and the wrappers.

## Running Locally

If you'd like to run on your local machine, edit `local_experiment.py` to match the environment setup and model loading with the one you have in `solution.py`, and run.
