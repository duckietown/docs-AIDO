# The AI Driving Olympics {#book:AIDO status=ready}

Maintainer: Andrea Censi, Liam Paull

<abbr>ML</abbr>, deep learning, and deep reinforcement learning have shown remarkable success on a variety of tasks in the very recent past. However, the ability of these methods to supersede classical approaches on  physically embodied agents is still unclear. In particular, it remains to be seen whether learning-based approached can be completely trusted to control safety-critical systems such as self-driving cars.

This live competition is designed to explore which approaches work  best for what tasks and subtasks in a complex robotic system. The participants will need to design algorithms that implement either part or all of the management and navigation required for a fleet of self-driving miniature taxis.

We call this competition the "AI Driving Olympics" because there will be a set of different trials that correspond to progressively more sophisticated behaviors for the cars. These  vary in complexity, from the reactive task of lane following to more complex and "cognitive" behaviors, such as obstacle avoidance, point-to-point navigation, and finally coordinating a vehicle fleet while adhering to the entire set of the "rules of the road". We will provide  baseline solutions for the tasks based on conventional autonomy architectures; the participants will be free to replace any or all of the components with custom learning-based solutions.

The competition will be live at NIPS, but participants will not need to be physically present---they will just need to send their source code packaged as a Docker image.

There will be qualifying rounds in simulation, similar to the recent DARPA Robotics Challenge,
and we will make available the use of "robotariums," which are facilities that allow remote experimentation in a reproducible setting.

**Keywords**: robotics, safety-critical AI, self-driving cars, autonomous mobility on demand, machine learning, artificial intelligence.
