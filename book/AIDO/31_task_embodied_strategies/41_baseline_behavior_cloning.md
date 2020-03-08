# Behavior Cloning {#embodied_bc status=ready}
In this part, you can find all the required steps in order to make a submission based on Behavior Cloning with Tensorflow for the lane following task using data varying from real data or simulator data. It can be used as a strong starting point for any of the [`LF`](#lf), [`LFV`](#lf_v), and [`LFVI`](#lf_v_i) challenges.

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

  4. *(Optional)* Hold all previous models you generated in `duckieModels` in case you need it.

  5. Submit your submission via `duckieChallenger` folder.


## The duckieSchool {#bc-duckieSchool status=ready}

## The duckieLog {#bc-duckieLog status=ready}










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
└── train.log                       # Your training data.
```
### Environment Setup {#bc-duckieTrainer-setup status=ready}

To setup your environment, I strongly urge you to train the model using a system with GPU. Tensorflow and GPU sometimes can be confusing, and I recommend you to refer to tensorflow documentation for detailed information.

Currently, the system requires `TensorFlow` 2.1.0. To setup TensorFlow, you can refer to the official TensorFlow install guide [here](https://www.tensorflow.org/install/gpu#ubuntu_1804_cuda_101).

Additionally, this training sytem utilizes `sklearn` and `numpy`.

### Model Adjustment {#bc-changeModel status=ready}

To change the model, you can modify the `frankModel.py` file as it includes the model architecture. Currently it uses a parallel architecture to seperately generate a linear and angular velocity. It might perform better if they are not setup seperately.

To change your training parameters, you can find EPOCHS, LEARNING RATE, and BATCH size at the beginning of `train.py`. You should tweak around these values with respect to your own provided training data.

    Note: For multi-gpu systems, you can enable multi-gpu training at training configuration section

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
