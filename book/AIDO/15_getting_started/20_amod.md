# AMOD Quickstart {#amod-quickstart status=draft}

<div class='requirements' markdown='1'>

Requires: You have [set up your accounts](#cm-accounts).

Requires: You have [the software requirement](#cm-sw).

Requires: You have [made a submission](#cm-first).

Result: You have made a submission to the AMOD challenge and you know how to try to make it better.

</div>

The objective of this quickstart guide is to present the template solutions provided for the AMoD - challenge and briefly explain what action they will take. First, please ensure that you have read the general information about the AMoD task [here](http://docs.duckietown.org/DT19/AIDO/out/amod.html). This page also contains instructions on how to obtain the provided template solutions and make a submission that implements these template solutions. 

## Implemented Logic in the Template Solutions

The current implementation (for instance visible in the file src/DispatchingLogic.py) does the following: 

* Open transporation requests by customers are sorted with respect to the submission time, i.e., the longest waiting customers are first in the list. 
* Next, for every unassigned customer request, a random robotic taxi is selected and sent to pickup the passenger and drive her or him to the desired destination.
* Finally, all robotic taxis which are not busy and in the task <I>STAY<I> are sent to a random location in the network (rebalancing step).

The same logic is also implmented in the JAVA template.


## How to make your score go up?

Of course, the implementation provided in the templates neglects a variety of very important aspects. Therefore, there are ways to improve its score rapidly, included but not limited to the following:

* Instead of sending **any** robotic taxi to pickup a customer, it might be good to send a close-by taxi or even minimize the total distance required to pickup all customers?
* Rebalancing to a random location might not be the best possible solution. How about stopping the rebalancing altogether? Or how about sending cars to **anticipated locations of future requests** ?

More inspiration can for instance be found when studying the benchmark operational policies implemented in [AMoDEUs](https://github.com/idsc-frazzoli/amodeus) and reading the respective scientific publications.
