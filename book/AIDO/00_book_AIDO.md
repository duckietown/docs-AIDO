# The AI Driving Olympics {#book:AIDO status=ready}

Maintainer: Andrea Censi, Liam Paull



<p style='text-align: center'>
  <img src="AIDO-768x512.png" width="60%"/>
</p>

<abbr>ML</abbr>, deep learning, and deep reinforcement learning have shown remarkable success on a variety of tasks in the very recent past. However, the ability of these methods to supersede classical approaches on  physically embodied agents is still unclear. In particular, it remains to be seen whether learning-based approached can be completely trusted to control safety-critical systems such as self-driving cars.

This live competition is designed to explore which approaches work  best for what tasks and subtasks in a complex robotic system. The participants will need to design algorithms that implement either part or all of the management and navigation required for a fleet of self-driving miniature taxis.

We call this competition the "AI Driving Olympics" because there will be a set of different trials that correspond to progressively more sophisticated behaviors for the cars. These  vary in complexity, from the reactive task of lane following to more complex and "cognitive" behaviors, such as obstacle avoidance, point-to-point navigation, and finally coordinating a vehicle fleet while adhering to the entire set of the "rules of the road". We will provide  baseline solutions for the tasks based on conventional autonomy architectures; the participants will be free to replace any or all of the components with custom learning-based solutions.

Participants will not need to be physically present---they will just need to send their source code packaged as a Docker image.  There will be qualifying rounds in simulation, similar to the recent DARPA Robotics Challenge,
and we will make available the use of "robotariums," which are facilities that allow remote experimentation in a reproducible setting.

**AIDO 1** is in conjuction with NIPS 2018. 

**AIDO 2** is in conjuction with ICRA 2019. 


## Leaderboards {#book-leaderboard nonumber notoc}

See the leaderboards at the site [`https://challenges.duckietown.org/`](https://challenges.duckietown.org) 
to check who is currently winning.

## Book organization {#book-org nonumber notoc}

[](#part:aido-introduction) provides a high-level overview of the scientific motivation and the various 
tasks.

[](#part:aido-rules) describes the logistics.

[](#part:manual) is a reference manual for setting up your environment.

[](#part:embodied) describes the embodied tasks.

[](#part:task-amod) describes the AMOD tasks.

[](#part:developers) containes information for challenges organizers.



<!--

### LF: Lane following 

<img style="width: 24em" src="https://challenges.duckietown.org/v3/humans/challenges/aido1_lf1-v3/leaderboard/image.png"/>

For more details, see [the online leaderboard](https://challenges.duckietown.org/v3/humans/challenges/aido1_lf1-v3/leaderboard).


### LF: Lane following + vehicles

Not online yet. 

### NAV: Navigation

Not online yet.


### AMOD: Simulated Autonomous Mobility on Demand


<img style="width: 24em" src="https://challenges.duckietown.org/v3/humans/challenges/aido1_amod1-v3/leaderboard/image.png"/>

For more details, see [the online leaderboard](https://challenges.duckietown.org/v3/humans/challenges/aido1_amod1-v3/leaderboard).
-->
