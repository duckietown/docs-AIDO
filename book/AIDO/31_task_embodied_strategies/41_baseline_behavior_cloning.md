# Behavior Cloning {#embodied_bc status=ready}

In this part, you can find all the required steps in order to make a submission based on Behavior Cloning with Tensorflow for the lane following task using data varying from real data or simulator data. It can be used as a strong starting point for any of the challenges.

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [tensorflow template](#tensorflow-template).

Result: You win the AI-DO!

</div>

## Introduction {#bc-whoami status=ready}

This baseline refers to Nvidia's approach for behvaior cloning for autonomous vehicles. You can find the original paper here: [End to End Learning for Self-Driving Cars](https://images.nvidia.com/content/tegra/automotive/images/2016/solutions/pdf/end-to-end-dl-using-px.pdf) It is created by [Frank (Chude Qian)](mailto:frank.qian@case.edu) for his submission to AIDO3 in NeurIPS 2019. The submission was very successful on simulator challenge, however, it was not the best for realworld challenges. I have decided to opensource this submission as a baseline to inspire better results. A detailed description on the specific implementation for this baseline can be find on the summary poster here: [Teaching Cars to Drive Themselves](https://doi.org/10.5281/zenodo.3660134) Additional reference can also be found on the summary poster.

## Quickstart {#bc-quickstart status=ready}

Clone the [baseline Behavior Cloning repository](https://github.com/duckietown/challenge-aido_LF-baseline-behavior-cloning):

    $ git clone -b daffy https://github.com/duckietown/challenge-aido_LF-baseline-behavior-cloning.git

    $ cd challenge-aido_LF-baseline-behavior-cloning

The code you find is structured into 5 folders.

1. Teach your duckiebot to drive itself in `duckieSchool`.

2. _(Optional)_ Store all the logs that can be used for training using `duckieLog`.

3. Train your model using tensorflow in `duickieTrainer`.

4. _(Optional)_ Hold all previous models you generated in `duckieModels` in case you need it.

5. Submit your submission via `duckieChallenger` folder.

## The duckieSchool {#bc-duckieSchool status=ready}

In side this folder you will find two types of duckieSchool: simulator based duckieGym and real robot based duckieRoad.

### Installing duckietown Gym {bc-duckieschool-gyminstall status=ready}

To install duckietown Gym and all the necessary dependencies:

    pip3 install --use-feature=2020-resolver -r requirements.txt

### Use joystick to drive

Before you use the script, make sure you have the joystick connected to your computer.

To run the script, use the following command:

    $ python3 human.py

The system utilizes an Xbox One S joystick to drive around. Left up and down controls the speed and right stick left and right controls the velocity. Right trigger enables the ["DRS" mode](https://en.wikipedia.org/wiki/Drag_reduction_system) allows vehicle to drive full speed forward. (Note there are no angular acceleration when this mode is enabled).

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

10. `--nb-episodes`: This controls how many episodes (aka sessions) you drive. This value typically don't matter as you will probably get tired before this value reaches.

11. `--logfile`: This specifies where you can store your log file. Defaultly it will just save the log file at the current folder.

12. `--downscale`: This option currently is disabled.

13. `--filter-bad-data`: This option allows you to only logs driving better than last state. It uses reward feedback on the duckietown gym for tracking the reward status.

Additionally, some other features has been hard coded:

1. Currently the training image are stored as YUV color space, you can change it in line 258.

2. Currently the frame is sized as 150x200 per Nvidia's recommendation. This could be not the most effective resolution.

3. Currently the logger resets if it detects you drive out of the bound.

### Automated log generation using pure pursuit

In this year, we also provide you with an option to automatically generate training sample using the concept of pure pursuit method. For more information, you can check out [this video](https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-geometric-lateral-control-pure-pursuit-44N7x)

The configurables are pretty much the same as the human driver agent.

If you would like to mass generate training samples on a headless server, under util folder you can find the tools for that.

To start pure pursuit data generation:

    $ python3 automatic.py

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
├── converted                       # Stores the converted log for you to train the duckiebot
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

https://docs.duckietown.org/daffy/duckietown-robotics-development/out/ros_logs.html

**You should change `extract_data.py` line 83 to the correct `VEHICLE_NAME`.**

First put your ROS bags in the bag_files folder. Then:

    $ make make_extract_container

Next start the conversion docker:

    $ make start_extract_data

It will automatically mount the bags folder as well as the converted folder.

**NOTE: When you run the make file, make sure you are in duckieRoad not in the src folder!**

## The duckieLog {#bc-duckieLog status=ready}

This folder is set for your to put all of your duckie logs. Some helper functions are provided. However, they might not be the most efficient ones to run. It is here for your reference.

### The log viewer

To view the logs, under duckieLog folder:

    $ python3 util/log_viewer.py --log_name YOUR_LOG_FILE_NAME.log

### The log combiner

To combine the logs, under duckieLog folder:

    $ python3 util/log_combiner.py --log1 dataset1.log --log2 dataset2.log --output newdataset.log

## The duckieTrainer {#bc-duckieTrainer status=ready}

This section describes everything you need to know using the duckieChallenger.

### Folder structure {#bc-duckieTrainer-folder status=ready}

In this folder you can find the following fils:

```
.
├── __pycache__                     # Python Compile stuff.
|
├── trainlogs                            # Training logs for tfboard.
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

Currently, the system requires `TensorFlow` 2.2.1. To setup TensorFlow, you can refer to the official TensorFlow install guide [here](https://www.tensorflow.org/install/gpu#ubuntu_1804_cuda_101).

Additionally, this training sytem utilizes `scikit-learn` and `numpy`. You can find a provided requirements.txt file that helps you install all the necessary packages.

    $ pip3 install -r requirements.txt

### Model Adjustment {#bc-changeModel status=ready}

To change the model, you can modify the `frankModel.py` file as it includes the model architecture. Currently it uses a parallel architecture to seperately generate a linear and angular velocity. It might perform better if they are not setup seperately.

To change your training parameters, you can find EPOCHS, LEARNING RATE, and BATCH size at the beginning of `train.py`. You should tweak around these values with respect to your own provided training data.

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
├── helperFncs.py                   # Helper file for all helper functions.
├── requirements.txt                # All required pip3 install.
├── solution.py                     # Your actual solution
└── submission.yaml                 # Submission configuration.
```

After you put your trained model `FrankNet.h5` in this folder, you can proceed as normal submission:

    $ dts challenges submit

Or run locally:

    $ dts challenges evaluate

An example submission looks like [this](https://challenges.duckietown.org/v4/humans/submissions/11410)

## Acknowledgement {#bc-acknowledge status=ready}

We would like to thank: [Anthony Courchesne](https://www.linkedin.com/in/courchesnea/) and [Kay (Kaiyi) Chen](mailto:kxc581@case.edu) for their help and support during the development of this baseline.
