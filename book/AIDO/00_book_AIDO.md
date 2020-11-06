# The AI Driving Olympics Urban League {#book:book status=ready}
 

<p style='text-align: center'>
  <img src="AIDO_no_text.png" width="60%"/>
</p>

## Welcome to Urban League of the AI Driving Olympics! {#aido-welcome nonumber notoc}

Despite recent breakthroughs, the ability of deep learning and reinforcement learning to outperform traditional approaches to control physically embodied robotic agents remains largely unproven. 

The [AI Driving Olympics][aido] (AI-DO) is a set of competitions with the objective of evaluating the state of the art in machine learning and artificial intelligence for mobile robotics. 

- **AIDO 1** was in conjunction with *NeurIPS 2018*.

- **AIDO 2** was in conjunction with *ICRA 2019*.

- **AIDO 3** was in conjunction with *NeurIPS 2019*.

- **AIDO 4** was supposed to be in conjunction with *ICRA 2020*, but was canceled due to COVID-19. 

- **AIDO 5** is in conjunction with *NeurIPS 2020*. 

The ``Urban League'' of the AI-DO is based on the [Duckietown platform][duckietown], and  includes a series of tasks of increasing complexity. For each task, we provide tools for competitors to use in the form of simulators, logs, code templates, baseline implementations and low-cost access to robotic hardware. We evaluate submissions in simulation online, on standardized hardware environments, and finally at the competition event.

<style>
  #variations th, #variations td {
    text-align: center;
  }
  #variations th {
    font-size: 120%;
  }
  #variations  td.explain {
    padding-left: 1em;
    text-align: left;
    vertical-align: top;
  } 
</style>

<table id="variations">
  <thead >
    <th><code>LF</code></th>
    <th><code>LFP</code></th>
    <th><code>LFV_multi</code></th>
  </thead>
  <tr>
    <td>Follow the lane</td>
    <td>Avoid the duckies pedestrians</td>
    <td>Control multiple Duckiebots</td>
  </tr>
  <tr>
    <td>
      <img src="https://i.imgur.com/sGTigpp.jpg" style='width: 10em'/>
    </td>
    <td>
      <img src="https://i.imgur.com/yNF8VDd.jpg" style='width: 10em'/>
    </td>
    <td>
      <img src="https://i.imgur.com/Ifr8ugB.jpg" style='width: 10em'/>
    </td>
  </tr>
</table>

Participants will not need to be physically present at any stage of the competition --- they will just need to send their source code.  
There will be qualifying rounds in simulation, similar to [recent DARPA Robotics Challenges](https://www.subtchallenge.com/), and, for evaluation, we  make available the use of "[Duckietown Autolabs](+opmanual_autolab#book-autolab)" which are facilities that allow remote experimentation in a reproducible setting. 


<br/>

<figure nonumber=1>
    <figcaption>Where it all started: AIDO 1 at NeurIPS 2018 in Montreal.</figcaption>
    <img style='width:30em' src="AIDO1.jpg"/>
</figure>



## What's new in AI-DO 5 {#whats-new nonumber}

There have been many cool new improvements for the 5 edition of the AI-DO Urban League:

- The robots in the simulators and the new DB19 Duckiebots are equipped with **encoders**.
- In the new `LFP` challenge there are now duckie-pedestrians to avoid.
- In the `LFV_multi` challenge you control **all** of the robots in a multi-vehicle setting, instead of just one. 
- The agents can now control the **LEDs** in the simulator.

TODO: link challenges.

Furthermore, there were many improvements to the back-end and baseline workflows to help you get started.


## How to use this documentation {#how-to-use nonumber}

If you would like to compete in the AI-DO Urban League, you will probably want to do something like:

- Read [the brief introduction to the competition](#part:aido-introduction) (~5 mins)
- Find [the challenge that you would like to try](#part:aido-rules) (~5 mins)
- [Get started and make a submission](#part:quickstart) (~5-20 mins depending on your setup)
 
At this point you are all setup, can make a submission, and you should want to make your submission better. To do this the following tools might prove useful:

- The [AIDO API](#part:manual) so that your workflow is efficient using our tools.
- The [reference algorithms](#part:embodied-strategies) where we have implemented some different approaches to solve the challenges.


## How to get help {#book-help nonumber}

If you are stuck try one of the following things:

- Look through the contents of this documentation using the links on the left. Note that the "Parts" have many "Chapters" that you can see when you click on the Part title,
- Join our [slack community](https://join.slack.com/t/duckietown/shared_invite/enQtNTU0Njk4NzU2NTY1LWM2YzdlNmJmOTg4MzAyODc2YTI3YTc5MzE2MThkZGUwYTFkZWQ4M2ZlZGU1YTZhYjg5YTgzNDkyMzI2ZjNhZWE),
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

[aido]: https://driving-olympics.ai/
[duckietown]: https://duckietown.org/
