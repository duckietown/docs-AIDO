# Imitation Learning in Simulation Workflow {#embodied_il_sim status=draft}

assigned: Manfred

## Supervised Learning (Imitation Learning)

Instead of learning purely through the reward signal you can also opt to teach your Duckiebot by being a good driver yourself. In reinforcement learning you are at some point learning a policy that takes an image as an input and outputs a an action. If you have data for this (combinations of images and correct driving signals/actions) then you can also train this policy in a supervised manner. This is called "imitation learning".

In order to pull this off, you need a good amount of data. You can get this data

- (A) by recording your own dataset, i.e. driving the duckiebot around (behavioral cloning) or following a more complex data gather approach like DAgger
- (B) by selecting and cleaning existing recordings from the Duckietown class that students provided

## A - Recording your own dataset

You can do this entirely in simulation. You can follow the instructions on the (quickstart page)[#gym-tutorial] up until and including the "Visual Debugging" section but instead of running `python3 agent.py` you run `python3 agent-keyboard-control.py` and by pressing <kbd>s</kbd> you can store the current observations and actions to a HDF5 file (more about this on the [Supervised/Imitation Learning page](#aido1-imitation-learning)).

You can run the script multiple times and every time it will append to the `recording/data.hdf5` file. You can analyze this file with the script `recording/analyze_data.py` - and by looking at the code of this script you can check how to read the HDF5 file back to Python (TODO: this paragraph's code isn't implemented yet).

## B - Selecting and cleaning existing data

Please head on over to http://logs.duckietown.org/ and either look at the GIFs or the MP4 videos of the recordings. You want to find recordings that are quite long, but not too long (because you will have to watch it in its entirety), that take place within the boundaries of Duckietown (as opposed to off-road) and that don't contain any crashes or collisions.

This last bit is crucial and you want to make sure you watch the entire recording to look for problems, because I contributed one of the recordings myself and at some point mid-recording I zoned out and drove the Duckie into a street sign. This can happen at any time in any recording.

Therefore the suggested steps are:

0. Create a new empty HDF5 dataset.
1. Select some random Duckiebot.
2. Check out the GIF. If it looks like the Duckie is offroad, go back to (1), otherwise continue.
3. Watch the whole MP4 video. Note the sections that you'd like to keep (i.e. that contain "good" driving). If the video has too few or no sections like this, go back to (1) otherwise continue.
4. Download and open the ROSbag corresponding to the recording.
5. Extract the time sections that you highlighted earlier, scale the images to have the same format as the simulator output: `120x160x3` and append the images and actions to the HDF5 file.
6. Go back to step (1) until you have a "reasonably" sized dataset.

Once you have this dataset, if you're following the "behavior cloning" approach, you can train a neural network to predict the expert's action from the input (image). This can be done with a simple convolutional neural network, where the output layer is `tanh`-activated.

Check out more over at our [Supervised/Imitation Learning page](#aido1-imitation-learning).

## How to train your model


## How to make your model into an AI-DO submission

---
