
# The AI Driving Olympics {#aido-welcome status=ready}

<!-- Despite recent breakthroughs, the ability of deep learning and reinforcement learning to outperform traditional approaches to control physically embodied robotic agents remains largely unproven.  -->

The [AI Driving Olympics][aido] (AI-DO) is a set of competitions with the objective of evaluating the state of the art in machine learning and artificial intelligence for mobile robotics. 

For a detailed description of the scientific objectives and outcomes please see [our recent paper about the AI-DO 1 at NeurIPS](https://arxiv.org/pdf/1903.02503.pdf).

<figure id="aido-4-video">
<figcaption>
The AI Driving Olympics at ICRA 2020
</figcaption>
<dtvideo src="vimeo:629305710"/>
</figure>
<minitoc/>

## History {#aido-welcome-history}


- **AI-DO 1** was in conjunction with **NeurIPS 2018**.

- **AI-DO 2** was in conjunction with **ICRA 2019**.

- **AI-DO 3** was in conjunction with **NeurIPS 2019**.

- **AI-DO 4** was supposed to be in conjunction with **ICRA 2020**, but was canceled due to COVID-19. 

- **AI-DO 5** was in conjunction with **NeurIPS 2020**.

- **AI-DO 6** is in conjunction with **NeurIPS 2021**.


<figure nonumber="1">
    <figcaption>Where it all started: AI-DO 1 at NeurIPS 2018 in Montreal.</figcaption>
    <img style='width:20em' src="AIDO1.jpg"/>
</figure>

## Leagues

There are currently three leagues in the AI Driving Olympics.

The **Urban League** is based on the [Duckietown platform][duckietown], and  includes a series of tasks of increasing complexity. For each task, we provide tools for competitors to use in the form of simulators, logs, code templates, baseline implementations and low-cost access to robotic hardware. We evaluate submissions in simulation online, on standardized hardware environments, and finally at the competition event.

Participants will not need to be physically present at any stage of the competition --- they will just need to send their source code.  
There will be qualifying rounds in simulation, similar to [recent DARPA Robotics Challenges](https://www.subtchallenge.com/), and, for evaluation, we  make available the use of "[Duckietown Autolabs](+opmanual_autolab#book)" which are facilities that allow remote experimentation in a reproducible setting. 

See the leaderboards and many other things at [the challenges site](https://challenges.duckietown.org).


The **Advanced Perception League** is organized by Motional (ex nuTonomy, Aptiv Mobility).
This book describes the urban league. All information about the Advanced Perception
League is at [nuScenes.org](https://nuscenes.org).

The **Racing League** is organized by the AWS Deepracer team. All information about the racing league is available
on [aicrowd.com](https://www.aicrowd.com/challenges/neurips-2021-aws-deepracer-ai-driving-olympics-challenge)


## What's new in the Urban League in AI-DO 6 {#whats-new}



There have been many cool new improvements for the 6th edition of the AI-DO Urban League:

- The challenges are now compatible the new [DB21 Duckiebots](https://get.duckietown.com/products/duckiebot-db21-m) that 
have Jetson Nanos with GPUs and were used for the [Self-Driving Cars with Duckietown MOOC on EdX](https://www.edx.org/course/self-driving-cars-with-duckietown).



<style>
  #variations { 
    font-size: smaller;
  }
  #variations th, #variations td {
    text-align: center;
  }
  #variations th {
    font-size: 120%;
  }
  #variations td {
    padding-left: 1em;
    padding-right: 1em;
  }
  #variations  td.explain {
    padding-left: 1em;
    text-align: left;
    vertical-align: top;
  } 
</style>

<figure>
  <figcaption>
  The three challenges of AI-DO 5.
  </figcaption>
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
        <img src="LF.jpg" style='width: 10em'/>
      </td>
      <td>
        <img src="LFP.jpg" style='width: 10em'/>
      </td>
      <td>
        <img src="LFV_multi.jpg" style='width: 10em'/>
      </td>
    </tr>
  </table>
</figure>

## How to use this documentation {#how-to-use}

If you would like to compete in the AI-DO Urban League, you will probably want to do something like:

- Read [the brief introduction to the competition](#part:aido-introduction) (~5 mins).
- Find [the challenge that you would like to try](#part:aido-rules) (~5 mins).
- [Get started and make a submission](#part:quickstart) (~5-20 mins depending on your setup).
 
At this point you are all setup, can make a submission, and you should want to make your submission better. To do this the following tools might prove useful:

- The [AIDO API](#part:manual) so that your workflow is efficient using our tools.
- The [reference algorithms](#part:embodied-strategies) where we have implemented some different approaches to solve the challenges.


## How to get help {#book-help nonumber}

If you are stuck try one of the following things:

- Look through the contents of this documentation using the links on the left. Note that the "Parts" have many "Chapters" that you can see when you click on the Part title,
- Join our [slack community](https://join.slack.com/t/duckietown/shared_invite/enQtNTU0Njk4NzU2NTY1LWM2YzdlNmJmOTg4MzAyODc2YTI3YTc5MzE2MThkZGUwYTFkZWQ4M2ZlZGU1YTZhYjg5YTgzNDkyMzI2ZjNhZWE),
- Look on the [Duckietown Stack Overflow](https://stackoverflow.com/c/duckietown/) to see if someone already answered your question
- If you are sure you actually found a bug, file a github issue in the appropriate repo.


<!-- ## The challenges server {#book-leaderboard} -->


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

If you use the Duckietown platform in your work and want to cite it please use:
```
@INPROCEEDINGS{PaullICRA2017,
    author={Paull, Liam and Tani, Jacopo and Ahn, Heejin and Alonso-Mora, Javier and Carlone, Luca and Cap, Michal and Chen, Yu Fan and Choi, Changhyun and Dusek, Jeff and Fang, Yajun and Hoehener, Daniel and Liu, Shih-Yuan and Novitzky, Michael and Okuyama, Igor Franzoni and Pazis, Jason and Rosman, Guy and Varricchio, Valerio and Wang, Hsueh-Cheng and Yershov, Dmitry and Zhao, Hang and Benjamin, Michael and Carr, Christopher and Zuber, Maria and Karaman, Sertac and Frazzoli, Emilio and Del Vecchio, Domitilla and Rus, Daniela and How, Jonathan and Leonard, John and Censi, Andrea},
    booktitle={2017 IEEE International Conference on Robotics and Automation (ICRA)}, title={Duckietown: An open, inexpensive and flexible platform for autonomy education and research},
    year={2017},
    volume={},
    number={},
    pages={1497-1504},
```

[aido]: https://driving-olympics.ai/
[duckietown]: https://duckietown.org/
