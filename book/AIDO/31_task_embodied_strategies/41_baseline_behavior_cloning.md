# Behavior Cloning {#embodied_bc status=beta}

In this part, you can find all the required steps in order to make a submission based on Behavior Cloning with Tensorflow for the lane following task using data varying from real data or simulator data. It can be used as a strong starting point for any of the [`LF`](#challenge-LF), [`LFV`](#challenge-LF_v), and [`LFVI`](#challenge-LFVI) challenges.

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [tensorflow template](#tensorflow-template).

Result: You win the AI-DO!

</div>

## Introduction {#bc-whoami status=ready}

This baseline refers to Nvidia's approach for behvaior cloning for autonomous vehicles. You can find the original paper here: [End to End Learning for Self-Driving Cars](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) It is created by [Frank (Chude Qian)](mailto:frank.qian@case.edu) for his submission to AIDO3 in NeurIPS 2019. The submission was very successful on simulator challenge, however, it was not the best for realworld challenges. I have decided to opensource this submission as a baseline to inspire better results. A detailed description on the specific implementation for this baseline can be find on the summary poster here: [Teaching Cars to Drive Themselves](https://doi.org/10.5281/zenodo.3660134) Additional reference can also be found on the summary poster.

## Quickstart {#bc-quickstart status=ready}

Clone the [baseline Behavior Cloning repository](https://github.com/duckietown/challenge-aido_LF-baseline-behavior-cloning):

    $ git clone -b master https://github.com/duckietown/challenge-aido_LF-baseline-behavior-cloning.git

    $ cd challenge-aido_LF-baseline-behavior-cloning

The code you find is structured into 5 folders.

1. Teach your duckiebot to drive itself in `duckieSchool`.

2. Sift through all the logs that can be used for training using `duckieLog`.

3. Train your model using tf.keras based model in `duickieTrainer`.

4. _(Optional)_ Hold all previous models you generated in `duckieModels` in case you need it.

5. Submit your submission via `duckieChallenger` folder.

## The duckieSchool {#bc-duckieSchool status=ready}

In side this folder you will find two types of duckieSchool: simulator based duckieGym and real robot based duckieRoad.

### Installing duckietown Gym {bc-duckieschool-gyminstall status=ready}

To install duckietown Gym, please refer to the instructions [here](https://https://github.com/duckietown/gym-duckietown/tree/daffy)

    $ git clone https://github.com/duckietown/gym-duckietown.git -b daffy
    $ cd gym-duckietown
    $ pip3 install -e .

### Use joystick to drive

Before you use the script, make sure you have the joystick connected to your computer.

To run the script, use the following command:

    $ python3 human.py

The system utilizes an Xbox 360 joystick to drive around. Left up and down controls the speed and right stick left and right controls the velocity. Right trigger enables the ["DRS" mode](https://en.wikipedia.org/wiki/Drag_reduction_system) allows vehicle to drive full speed forward. (Note there are no angular acceleration when this mode is enabled).

In addition, every 1500 steps in simulator, the recording will pause and playback. You will have the chance to review the result and decide whether to keep the log or not. The log are recorded into two formats: `raw_log` saves all the raw information for future re-processing, and `traning_data` saves the directly feedable log.

### Options for joystick script

For driving duckiebot with a joystick in a simulator, you have the following options:

1. `--env-name`: currently the default is `None`.

2. `--map-name`: This sets the map you choose to run. Currently it is set as `small_loop_cw`.

3. `--draw-curve`: This draw the lane following curve. Defaultly it is set as `False`. However, if you are new to the system, you should familiarize yourself with enabling this option as `True`.

4. `--draw-bbox`: This helps draw out the collision detection bounding boxes. Defaultly it is set as `False`.

5. `--domain-rand`: This enables domain randomization. Defaultly it is set as `True`.

6. `--playback`: This enables playback after each record section for you to inspect the log you just took. Defaultly it is set as `True`.

7. `--distortion`: This enables distortion to let the view as fisheye lens. Defaultly it is set as `True`.

8. `--raw_log`: This enables recording also a high resolution version of the log instead of the downsampled version. Defaultly it is set as `True`. **Note: if you disable this option, playback will be disabled too.**

9. `--steps`: This sets how many steps to record once. Defaultly it is set as `1500`

Additionally, some other features has been hard coded:

1. Currently the logger only logs driving better than last state. It uses reward feedback on the duckietown gym for tracking the reward status.

2. Currently the training image are stored as YUV color space, you can fix it in line 258.

3. Currently the frame is sized as 150x200 per Nvidia's recommendation. This could be not the most effective resolution.

4. Currently the logger resets if it detects you drive out of the bound.

### Log using an actual duckiebot

To log using an actual duckiebot, refer to [this](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/take_a_log.html) tutorial on how to get a rosbag on a duckiebot.

Once you have obtianed the ROS bag, you can use the folder `duckieRoad` to process that log.

### Process a log from an actual duckiebot

You will find the following files in the `duckieRoad` directory.

```
.
├── Dockerfile                      # File that sets up the docker image
|
├── bag_files                       # Put your ROS bags here.
│   ├── ROSBAG1                     # Your ROS bag.
│   ├── ROSBAG2                     # Your training on Date 2.
│   └── ...
|
├── src                             # Scripts to convert ROS bag to pickle log
│   ├── _loggers.py                 # Logger used to log the pickle log
│   ├── extract_data_functions.py   # Helper function for the script
│   └── extract_data.py             # Convertion script. You set your duckiebot
|                                     name, and topic to convert here.
|
├── MakeFile                        # Make file.
├── requirements.txt                # Used for docker to setup dependency
└── pickle23.py                     # Convert the pickle2 style log produced to                                    pickle 3
```

**You should change `extract_data.py` line 83 to the correct `duckiebot_name`.**

First put your ROS bags in the bag_files folder. Then:

    $ make docker_extract_data

This should automatically create the logs desired. Once you see the container loops `Get Log!!!`, that means all convertions are done. Then first find your running docker:

    $ docker container ls

Once found, then:

    $ docker cp YOUR_CONTAINER_NAME:/workspace/data ../bag2log

Rename the log file as `training_data.log` and put it same level as `pickle23.py` and convert the Pickle 2 style log to usable pickle 3 style:

    $ python3 pickle23.py

You should be able to get usable real robot log named `converted_from_pickle2.log`.

## The duckieLog {#bc-duckieLog status=ready}

This folder is set for your to put all of your duckie logs. Some helper functions are provided. However, they might not be the most efficient ones to run. It is here for your reference.

`combiner.py` helps you combine different logs together.

`pickle23.py` helps you convert pickle 2 log into pickle 3 log.

## The duckieTrainer {#bc-duckieTrainer status=ready}

This section describes everything you need to know using the duckieChallenger.

### Folder structure {#bc-duckieTrainer-folder status=ready}

In this folder you can find the following fils:

```
.
├── __pycache__                     # Python Compile stuff.
|
├── logs                            # Training logs for tfboard.
│   ├── Date 1                      # Your training on Date 1.
│   ├── Date 2                      # Your training on Date 2.
│   └── ...
|
├── trainedModel                    # Your trained model is here.
│   ├── FrankNetBest_Loss.h5        # Lowest training loss model.
│   ├── FrankNetBest_Validation.h5  # Lowest validation loss model.
│   └── FrankNet.h5                 # The last model of the training.
|
├── frankModel.py                   # The deep learning model.
├── logReader.py                    # Helper file for reading the log
├── train.py                        # The training setup.
├── requirements.txt                # Required pip3 packges for training
└── train.log                       # Your training data.
```

### Environment Setup {#bc-duckieTrainer-setup status=ready}

To setup your environment, I strongly urge you to train the model using a system with GPU. Tensorflow and GPU sometimes can be confusing, and I recommend you to refer to tensorflow documentation for detailed information.

Currently, the system requires `TensorFlow` 2.2.0. To setup TensorFlow, you can refer to the official TensorFlow install guide [here](https://www.tensorflow.org/install/gpu#ubuntu_1804_cuda_101).

Additionally, this training sytem utilizes `scikit-learn` and `numpy`. You can find a provided requirements.txt file that helps you install all the necessary packages.

    $ pip3 install -r requirements.txt

### Model Adjustment {#bc-changeModel status=ready}

To change the model, you can modify the `frankModel.py` file as it includes the model architecture. Currently it uses a parallel architecture to seperately generate a linear and angular velocity. It might perform better if they are not setup seperately.

To change your training parameters, you can find EPOCHS, LEARNING RATE, and BATCH size at the beginning of `train.py`. You should tweak around these values with respect to your own provided training data.

    Note: For multi-gpu systems, you can enable multi-gpu training at training configuration section

    Note: Multi-gpu training is an experimental feature. If you encounter any bug, please report it.

### Before Training {#bc-duckieTrainer-beforeStart status=ready}

Before you start training, make sure your log is stored at the root of the `duckieTrainer` folder. It should be named as `train.log`.

Make sure you have saved all the desired trained models into duckieModels. Trust me you do not want your overnight training overwritten by accident. Yes I have been through losing my overnight training result.

### Train it {#bc-duckieTrainer-trainnow status=ready}

To train your model:

    $ python3 train.py

To observe using tensorboard, run this command in the `duckieTrainer` directory:

    $ tensorboard --logdir logs

You should be able to also see your training status at `http://localhost:6006/`. If your computer is accessible by other computers, you can also see it by visiting `http://TRAINERIP:6006`

### Things to improve {#bc-duckieTrainer-improve status=ready}

There are a lot of things could be improved as this is an overnight hack for me. The data loading could be maybe more efficient. Currently it just load all and stores all in a global variable. The training loss reference might not be the best. The optimizeer might be improved. And most importantly, the way of choosing which model to use could be drastically improved.

### Troubeshooting {#bc-duckieTrainer-troubleshoot status=ready}

    Symptom: tensorflow.python.framework.errors_impl.InternalError: CUDA runtime implicit initialization on GPU:0 failed. Status: out of memory

    Resolution: Currently there is no known fix other than cross your fingers and run again and reducing your batch size.

## The duckieModels {#bc-duckieModels status=ready}

This is a folder created just for you to keep track of all your potential models. There is nothing functional in it.

## The duckieChallenger {#bc-duckieChallenger status=ready}

This is the folder where you submit to challenge. The folder is structured as follows: 

```
.
├── Dockerfile                      # Docker file used for compiling a container.
|                                     Modify this file if you added file, etc.
├── Dockerfile_LEGACY               # Legacy docker setting for models generated 
|                                     prior to Tensorflow 2.0 
|
├── helperFncs.py                   # Helper file for all helper functions.
├── requirements.txt                # All required pip3 install.
├── requirements.txt_LEGACY         # All required pip3 install for using with TF 1.x.
├── solution.py                     # Your actual solution
└── submission.yaml                 # Submission configuration.
```

After you put your trained model `FrankNet.h5` in this folder, you can proceed as normal submission:

    $ dts challenges submit

Or run locally:

    $ dts challenges evaluate

If you are attempting a Tensorflow model that is trained and generated using Tensorflow≤2.0 you should switch the `Dockerfile` and `requirements.txt` with the two files that is ended with `_LEGACY` to prevent version issue. By switching to legacy version, you will use Tensorflow 1.15 and Keras 2.3.1. 
