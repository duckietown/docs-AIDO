# The Classical Duckietown Software Stack {#embodied_classic status=ready}

## Getting the Docker images

Finding and pulling the right Docker images is documented [here](#ros-baseline). Depending on your use case, you may want to start from either the Lane Following [baseline repo](https://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown) or the random agent ROS [template repo](https://github.com/duckietown/challenge-aido1_LF1-template-ros).

## Modifying Launch Files

You can easily write your own launch files for your nodes. You will want to modify your equivalent of `lf_slim.launch` in the [baseline repo](https://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown), and make sure your `rosagent.py` is updated to listen to the correct topics. Remember, the challenges take `action = [vel_left, vel_right]` as input, so be sure to convert your actions using something like the `inverse_kinematic_node` if necessary.

## Modifying Parameters

What is hosted in the both the template and example repos are just starting points - you will need to make modifications in order to score well on the challenges. In order to do this, you will need to make sure you copy, build, and source your `catkin_ws` (or wherever you are putting your ROS code). Doing so is described in the [baseline repo](https://github.com/duckietown/challenge-aido1_LF1-baseline-duckietown), inside of the `Dockerfile`.

Once your `catkin_ws` is built and sourced, your regular Python code need only import the package, or send messages on the corresponding topic name. Templates on how to subscribe and publish to corresponding topics can be found inside of the `rosagent.py` code.

## Making an AI-DO Submission

To ensure a working submission, you can use the `dts challenges [evaluate | submit]` commands to run your evaluation locally or on a server (respectively). To ensure correct scoring, model your `solution.py` and `submission.yaml` after the ones in the previously-mentioned repositories.

