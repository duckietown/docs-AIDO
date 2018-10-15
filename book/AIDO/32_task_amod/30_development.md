# Development guide {#amod-devel-guide status=ready}

Assigned: Claudio, Jan

## Protocol {status=ready}

The protocol of these challenges is well-documented at

https://github.com/idsc-frazzoli/amod/blob/master/doc/aido-client-protocol.md

Read it carefully at this location before getting started. A brief qualitative explanation of the contained steps is presented in the following subsections.

Furthermore a JAVA and a Python baseline are available at these locations:

[the repository `challenge-aido1_amod1-template-java`][challenge-aido1_amod1-template-java]
[`challenge-aido1_amod1-template-python`][challenge-aido1_amod1-template-python]


### Pre-Execution Steps

Before a simualtion scenario is started, the user neeeds to select a scenario, e.g.,  SanFrancisco.20080518, which represents a scenario based on taxi trips recorded in SanFrancisco on 18th of May, 2008. Other scenarios include TelAviv, Santiago, Berlin or other days of San Fancisco taxi demand.

The Server responds with the total number of requests in the scenario, a bouding box in WGS84 coordinates and the nominal fleet size to serve all requests. 

Then, the client, i.e., the Aido-Guest can respond with the desired number of requests and the desired fleet size. 


### Simulation Execution

In the main loop of a simulation, the Aido-Guest client continuously receives information on location of open customer requests and the status of the fleet, e.g., vehicles locations and statuses. 

The client then needs to direct the action of the fleet, i.e., assign avaialble vehicles to pickup requests and relocate empty vehicles to improve performance. 


### Post-Execution Steps

Finally, the client responds with the final score creates detailed overviews of the achieved performance including a detailed fleet performance report in HTML. 


## How to create your own submission {status=draft}

TODO: to write
