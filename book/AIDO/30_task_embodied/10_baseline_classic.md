# ROS Template {#ros-template status=ready}

This section describes the basic procedure for making a submission with a model trained in using the [Robot Operating System](http://www.ros.org/).

<div class='requirements' markdown='1'>

Requires: That you have setup your [accounts](#cm-accounts).

Requires: That you meet the [software requirement](#cm-sw).

Result: You make a submission and see your entry on the [Leaderboard](https://challenges.duckietown.org/).

</div>



To create a submission for ROS, there are a few templates that you can start from. 

## Random ROS Baseline {#random-ros-baseline status=ready}

We will start from the Random ROS Baseline. This image extends `duckietown/rpi-duckiebot-base:master18`, which means that you will be able to run this exact code with the both the simulator and Duckiebot.


Clone the [template repo](https://github.com/duckietown/challenge-aido1_LF1-template-ros):

    $ git clone git@github.com:duckietown/challenge-aido1_LF1-template-ros.git

Enter the repo:

    $ cd challenge-aido1-LF1-template-ros

Either make a submission with:

    $ dts challenges submit


Or, run local evaluation with:

    $ dts challenges evaluate
    

Once this finishes, you'll receive a link where you can follow the progress of your evaluation. Once it's done check out how you did on the  [Leaderboard](https://challenges.duckietown.org/).
