# Imitation Learning from Logs Baseline (tensorflow) {#embodied_il_logs status=ready}

In this part, you can find all the required steps in order to make a submission based on Imitation Learning with Tensorflow for the lane following task using real log data .

<div class='requirements' markdown='1'>

Requires: That you have made a submission with the [tensorflow template](#tensorflow-template).

Result: You win the AI-DO!

</div>



## Quickstart {#tensorflow-from-logs-workflow status=ready}

Clone the [baseline Tensorflow imitation learning from logs repository](https://github.com/duckietown/challenge-aido1_LF1-baseline-IL-logs-tensorflow):

    $ git clone https://github.com/duckietown/challenge-aido1_LF1-baseline-IL-logs-tensorflow.git
    $ cd challenge-aido1_LF1-baseline-IL-logs-tensorflow

The code you find is structured into 3 folders.

  1. Extracting data

  2. Learning from the data

  3. Submitting learned model

Steps 1 and 2 can be run either using Docker or without.

### Extract data {#tensorflow-from-logs-data status=ready}

Go to the `extract_data` folder:

    $ cd extract_data

#### Docker data extraction
If using Docker (to avoid a ROS installation), type:

    $ make docker_extract_data

Once extraction is completed, move the extracted data to the learning folder. Type:

    $ make docker_copy_for_learning

#### ROS data extraction

If you already have ROS on your system, you may be able to not use Docker. Then type:

    $ make install-dependencies

To install some dependencies in a Python virtual environment.

To extract the data, type:

    $ make regular_extract_data

Once extraction is completed, move the extracted data to the learning folder. Type:

    $ make regular_copy_for_learning

### Learning {#tensorflow-from-logs-learning status=ready}

Go to the `learning` folder

    $ cd ../learning

#### Docker learning
If using Docker (to avoid a ROS installation), type:

    $ make learn-docker

to train a small convolutional neural network for imitation learning.
This assumes that you have extracted data in the previous step.

Once learning is completed, move the learned model to the submission folder. Type:

    $ make copy_for_submission

#### ROS learning

If you already have your Tensorflow learning pipeline on your system setup and do not use Docker. Then type:

    $ make install-dependencies

To install some dependencies in a Python virtual environment.

To train a small convolutional neural network for imitation learning, type:

    $ make learn-regular

Once learning is completed, move the learned model to the submission folder. Type:

    $ make regular_copy_for_submission

### Submission {#tensorflow-from-logs-submission status=ready}

If you have completed the previous steps, you will be able to submit this as is, with:

    $ dts challenges submit


Or, run local evaluation with:

    $ dts challenges evaluate


## Details and Workflow


### Changing the code

You may change the code to your needs. The code is located in the `src` folders in the respective folders.

#### Neural network design

In particular you can adapt the neural network model in `learning/src/cnn_training_functions.py`.

The type of neural network, its architecture and hyperparameters are choices that you are asked to make, but as a baseline a CNN model which takes as inputs images and predicts angular velocities is provided. This model is not only relatively simple but also takes as input low resolution images indicating that extremely complex models may not necessarily be required to solve the task of lane following without dynamic objects. In any case, during training it is necesary to adhere to the following actions:

1. name all your layers in order to be able to tell later which is the output layer of your neural network
2. prepare model for mobile deployment generating a `graph.pb` file (TensorFlow GraphDef file in binary format)


#### Logs selection

Which logs are downloaded is specified at the bottom of `extract_data/src/download_logs.py`.

Such data are saved in `.bag` files which are available in [Duckietown Logs Database](http://logs.duckietown.org). 
The important note here is to feed your agent with *appropriate* data. Appropriate log data are considered those which:

1. are relevant to the lane following task
2. execute this task well the whole time 
3. and present smooth driving of the duckiebots around the city.

One hint, is to search for data which were collected using the lane controller. In order to check if the lane controller was enabled, use rqt_bag to see if there is inside the node `/duckiebot_name/lane_controller/`. The *LF_IL_tensorflow* baseline, provides 5 bag files with approximately 10 minutes of appropriate lane following data.

#### How to prepare your data for training

TODO: Panos is this neccessary?

When working with real log data, this can be splitted into the following two subtasks:

1. extract desired topics from bag files
2. synchronize pairs of data from different topics

The first part should be clear, so let us focus to the second one. ROS messages from different topics are neither always of the same number nor properly arranged in the bag files. In this implementation the topics of interest are: 

- /camera\_node/camera/compressed
- /lane\_controller\_node/car\_cmd

and the aforementioned problem is visualized in Figure \ref{fig:synch}. Although a continuous sequence of images and car commands is expected, images and/or velocities could be omitted or even saved in slighlty translated timestamps. For this reason, in order to ensure the validity of our pairs of data, a synchronization is necessary.


<div figure-id="fig:synch">
<img src="images/synchronization_issue.png" style='width: 90%'/>
</div>


It is stated that in this case the synchronization is based on the fact that when using the lane controller images cause the car commands and not the other way around, while between two consecutive images there should be only one car command. For your convenience, in the provided baseline there is a script that takes care of these two steps for you and eventually saves the images with their respective car commands to HDF5 files. 


#### Image preprocessing

Additionally, the way images are preprocessed is specified in `extract_data/src/extract_data_functions.py`
and `learning/src/cnn_predictions.py`.

### A submission with the Movidius Neural Compute Stick

At this point your model is trained and you are asked to compile it to a Movidius graph. First, you should freeze the TensorFlow graph. This procedure consists of the following steps:

- combine the TensorFlow graph of your CNN model and its weights in a single file
- convert variables into inline constants
- given the output node of your output layer, keep only those operations that are necessary for the predictions
- get rid of all training node
- and apply optimization flags.

The script `/learning/src/freeze_graph.py` is part of the baseline and can be your guide for this procedure. The only tricky part here is to define correctly the output node which should not be confused with the output layer. In general, the output node should be easily found by the following name `scope_name(if exists)/output_layer_name/BiasAdd`.  

Once you have the frozen graph, you are only one step away from the Movidius graph. The last step is to [install NCSDK v2.05](https://movidius.github.io/ncsdk/index.html) in order to use its SDK tools to compile the TensorFlow frozen graph to a Movidius graph. After installing NCSDK v2.05, you just have to execute the command:
    
    $ mvNCCompile -s 12 path_to_frozen_graph.pb -in input_node_of_TensorFlow_model -on output_node_of_TensorFlow_model -o path_to_save_the_movidius_graph.graph 


