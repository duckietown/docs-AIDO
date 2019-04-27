# Agent Template for `AMOD` {#amod-baselines status=ready}

If you have not done this yet, study the general introduction to the task and the explanation of the performance objectives on these two pages:

- [](#amod)
- [](#performance)

Please briefly study the our client-server protocol which is described in our github repository (link below). It details the communication between the Aido-Host container in which the autonomous mobility-on-demand (AMoD) simulation is running and the Aido-Guest container in which you can implement your fleet operational policy:

> [aido-client-protocol.md](https://github.com/idsc-frazzoli/amod/blob/master/doc/aido-client-protocol.md)

Next, you can use one of the baselines that we provided to understand how an Aido-Guest to manage an AMoD task is actually implemented. We provide one baseline in JAVA and one in Python. Both do not represent well-performing policies but are simply demonstrations of how an Aido-Guest could be implemented, therefore you should rapidly be able to beat their performance. 

## Java

The Java baseline is available at [the repository `challenge-aido_amod-template-java`][challenge-aido1_amod1-template-java].

## Python

The Python baseline is available at the repository [`challenge-aido_amod-template-python`][challenge-aido1_amod1-template-python].

[challenge-aido1_amod1-template-python]: https://github.com/duckietown/challenge-aido1_amod1-template-python

[challenge-aido1_amod1-template-java]: https://github.com/duckietown/challenge-aido1_amod1-template-java
