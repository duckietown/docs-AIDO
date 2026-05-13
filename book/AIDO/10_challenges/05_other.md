# General rules {#other-rules status=ready}  

<minitoc />

## Protocol

### Deployment technique

We use Docker containers to package, deploy, and run the applications on the physical Duckietown platform as well as on the cloud for simulation. Base Docker container images are provided and distributed via [Docker HUB][dockerhub].

[dockerhub]: https://hub.docker.com/r/duckietown/

A **challenges server** is used to collect and queue all submitted agents. The **simulation evaluations** execute each queued agents as they become available. Submissions that pass the simulation environment will be queued for execution in the Autolab.

<div figure-id="fig:aido-submissions-workflow" figure-caption="The AI-DO evaluations workflow supports local and remote development, in simulation and on hardware.">
     <img src="dn-arch.png" style='width: 20em' />
</div>

For validation of submitted code and evaluation the competition finals a surprise environment will be employed. This is to discourage over-fitting to any particular Duckietown configuration.


### Submission of entries

Participants can submit their code in the form of a docker container to a challenge. Templates are provided for creating the container image in a conforming way.

The system will schedule to run the submitted robot agent on the cloud on the challenges selected by the user, and, if simulations pass, in the Autolabs.

Participants can submit entries as many times as they would like, which will be processed on a best effort basis. Access control and prioritization policies are in place to provide equal opportunities to all participants and prevent monopolization of the computational and physical resources available.

Participants are required to open source their solutions source code. If auxiliary training data are used to train the models, that data must be made available.

Submitted code will be evaluated in simulation and if sufficient on physical Autolabs. Scores and logs generated with submitted code are made available on the challenges server. 

### Simulators {status=beta}

[Historical simulation code](https://github.com/duckietown/gym-duckietown/) is available as open source for everybody to use on computers that they control. In the current `ente` repo set documented in this book, the maintained public evaluation path uses Duckiematrix and the active LF challenge definition rather than the older standalone Gym quickstart.

### Autolab test and validation

<!-- If there are $n$ robotariums available, $n-1$ robotariums can be used for training and testing, while 1 robotarium is used for validation. -->

When an experiment is run in a **training/testing** Autolab, participants historically received detailed feedback, including logs, telemetry, and videos. In the current `ente` repo set documented here, the maintained public workflow is the Duckiematrix simulation queue exposed on [the staging challenges server](https://staging-challenges.duckietown.com/).

<div figure-id="fig:lf-demo-ttic" figure-caption="Autolab LF-challenge evaluation demo.">
<dtvideo src="vimeo:561305335"/>
</div>

For public validation on `ente`, the staging challenges UI reports the summary metrics for the relevant evaluation step, while richer debugging remains available through local runs with `dts challenges evaluate`.

### Leaderboards

After each run, participants can see the metrics statistics on the [staging challenges website](https://staging-challenges.duckietown.com/). Per-challenge leaderboards are available there for the maintained public challenges, including [`aido-LF-sim-validation`](https://staging-challenges.duckietown.com/humans/challenges/aido-LF-sim-validation/leaderboard).


## Eligibility

Employees and affiliates of organizing and sponsoring organizations are ineligible from participation in the competition, but they are welcome to submit baseline solutions that will be reported in a special leaderboard.

Students of organizing institutions (ETH Zürich, University of Montreal, and TTIC), are eligible to participate in the competition as part of coursework, if they do not work in the organization of the competition.

## Intellectual property

Participants of AI-DO are required to provide the source code / data / learning models of their submission to the organizers before the finals (so that we can check for their regularity.)

Winners of AI-DO are required to make their submission open source so that
it can be reused later in the next challenges.