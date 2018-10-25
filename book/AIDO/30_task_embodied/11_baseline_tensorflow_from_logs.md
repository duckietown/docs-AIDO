# Tensorflow from logs baseline {#tensorflow-from-logs-baseline status=ready}

To create an imitation learning submission using Tensorflow learning from logs use this baseline.


## Workflow {#tensorflow-from-logs-workflow status=ready}

Clone the [baseline Tensorflow imitation learning from logs repository](https://github.com/duckietown/challenge-aido1_LF1-baseline-IL-logs-tensorflow):

    $ git clone https://github.com/duckietown/challenge-aido1_LF1-baseline-IL-logs-tensorflow.git
    $ cd challenge-aido1_LF1-baseline-IL-logs-tensorflow

The code you find is structured into 3 folders.

  1. Extracting data

  2. Learning from the data

  3. Submitting learned model

Steps 1 and 2 can be run either using Docker or without.

## Extracting data {#tensorflow-from-logs-data status=ready}

Go to the `extract_data` folder:

    $ cd extract_data

### Docker data extraction
If using Docker (to avoid a ROS installation), type:

    $ make docker_extract_data

Once extraction is completed, move the extracted data to the learning folder. Type:

    $ make docker_copy_for_learning

### Regular ROS data extraction

If you already have ROS on your system, you may be able to not use Docker. Then type:

    $ make install-dependencies

To install some dependencies in a Python virtual environment.

To extract the data, type:

    $ make regular_extract_data

Once extraction is completed, move the extracted data to the learning folder. Type:

    $ make regular_copy_for_learning

## Learning {#tensorflow-from-logs-learning status=ready}

Go to the `learning` folder

    $ cd ../learning

### Docker learning
If using Docker (to avoid a ROS installation), type:

    $ make learn-docker

to train a small convolutional neural network for imitation learning.
This assumes that you have extracted data in the previous step.

Once learning is completed, move the learned model to the submission folder. Type:

    $ make copy_for_submission

### Regular learning

If you already have your Tensorflow learning pipeline on your system setup and do not use Docker. Then type:

    $ make install-dependencies

To install some dependencies in a Python virtual environment.

To train a small convolutional neural network for imitation learning, type:

    $ make learn-regular

Once learning is completed, move the learned model to the submission folder. Type:

    $ make regular_copy_for_submission

## Submission {#tensorflow-from-logs-submission status=ready}

If you have completed the previous steps, you will be able to submit this as is, with:

    $ dts challenges submit


Or, run local evaluation with:

    $ dts challenges evaluate


## Changing the code

You may change the code to your needs. The code is located in the `src` folders in the respective folders.

### Neural network design

In particular you can adapt the neural network model in `learning/src/cnn_training_functions.py`.

Furthermore using different ROS bags to learn from may be of interest.

### Logs selection

Which logs are downloaded is specified at the bottom of `extract_data/src/download_logs.py`.


### Image preprocessing

Additionally, the way images are preprocessed is specified in `extract_data/src/extract_data_functions.py`
and `learning/src/cnn_predictions.py`.
