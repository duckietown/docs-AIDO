# The AI Driving Olympics {#book:AIDO status=ready}
 

<p style='text-align: center'>
  <img src="AIDO_no_text.png" width="60%"/>
</p>

## Welcome to AI Driving Olympics! {#aido-welcome nonumber notoc}

Despite recent breakthroughs, the ability of deep learning and reinforcement learning to outperform traditional approaches to control physically embodied robotic agents remains largely unproven. 
The “AI Driving Olympics” (AI-DO) is a competition with the objective of evaluating the state of the art in machine learning and artificial intelligence for mobile robotics. 
Based on the Duckietown platform, AI-DO includes a series of tasks of increasing complexity – from simple lane-following to fleet management. 
For each task, we provide tools for competitors to use in the form of simulators, logs, code templates, baseline implementations and low-cost access to robotic hardware. We evaluate submissions in simulation online, on standardized hardware environments, and finally at the competition event.

Participants will not need to be physically present at any stage of the competition --- they will just need to send their source code packaged as a Docker image.  There will be qualifying rounds in simulation, similar to the recent DARPA Robotics Challenge, and we will make available the use of "robotariums," which are facilities that allow remote experimentation in a reproducible setting. New to the AI-DO 2 edition is onsite testing on the competition grounds in the days leading up to the final.

 - **AIDO 1** is in conjunction with NeurIPS Dec. 2018. 

 - **AIDO 2** is in conjunction with ICRA May 2019.

 - **AIDO 3** is in conjunction with NeurIPS Dec 2019.


<figure>
    <figcaption>The AIDO 1 at NeurIPS in Montreal</figcaption>
    <img style='width:30em' src="AIDO1.jpg"/>
</figure>


## How to use this documentation {#how-to-use nonumber}

If you would like to compete in the AI-DO, you will probably want to do something like:

 - Read [the brief introduction to the competition](#part:aido-introduction) (~5 mins)
 - Find [the challenge that you would like to try](#part:aido-rules) (~5 mins)
 - [Get started and make a submission](#part:quickstart) (~5-20 mins depending on your setup)
 
 At this point you are all setup, can make a submission, and you should want to make your submission better. To do this the following tools might prove useful:
 
 - The [AIDO API](#part:manual) so that your workflow is efficient using our tools.
 - The [reference algorithms](#part:embodied-strategies) where we have implemented some different approaches to solve the challenges.


## How to get help {#book-help nonumber}

If you are stuck try one of the following things:

 - Look through the contents of this documentation using the links on the left. Note that the "Parts" have many "Chapters" that you can see when you click on the Part title,
 - Join our [slack community](https://join.slack.com/t/duckietown/shared_invite/enQtNTU0Njk4NzU2NTY1LTQ2MDI4MTY1OTE1YjhjMTU4YTdkMDViMzJmNmJkNGQxN2U1ZGJlZjk2NGM0M2FiODY3YmQ2MTQ3MGM2MjY1ZTI),
 - If you are sure you actually found a bug, file a github issue in the appropriate repo.


## The challenges server {#book-leaderboard nonumber}

See the leaderboards and many other things at [the challenges site](https://challenges.duckietown.org).


## How to cite {#how-to-cite nonumber notoc}

If you use the AI-DO platform in your work and want to cite it please use:

```
@article{zilly2019ai,
  title={The AI Driving Olympics at NeurIPS 2018},
  author={Julian Zilly and Jacopo Tani and Breandan Considine and Bhairav Mehta and Andrea F. Daniele and Manfred Diaz and Gianmarco Bernasconi and Claudio Ruch and Jan Hakenberg and Florian Golemo and A. Kirsten Bowser and Matthew R. Walter and Ruslan Hristov and Sunil Mallya and Emilio Frazzoli and Andrea Censi and Liam Paull},
  journal={arXiv preprint arXiv:1903.02503},
  year={2019}
}
```
