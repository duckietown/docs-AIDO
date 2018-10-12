# Development guide {#amod-devel-guide status=ready}

Assigned: Claudio, Jan

## Protocol {status=ready}

The challenge is accessed via a *Python* commands that are briefly outlined in pseudo-code in this section. Furthermore, the standard AMoDeus commands in *JAVA* can be used, although this is not necessary to participate in the challenge.

### Pre-Execution Steps

The first command is used to specify all settings for the simulation, i.e., if a specific scenario should be chosen, if $N_B$ or custom fleet sizes are chosen and if the full population should be reduced to a percentage $p$. Finally the number of simulation runs can be specified.

    void setSimulationSettings(ScenarioName, ...Optional N, Optional p )

The second command is used to retrieve all static information for the chosen case.

    (network, N_B, N, K, p) = getSimulationSettings()


### Simulation Execution

The first command can be used to access all simulation information live:


    (sigma(t), sigma_bar(t), robotaxi information, ... request information) = // getStatus()

The second command is used to send a robotaxi to pickup a customer:

    void setRoboTaxiPickup(Robotaxi, Request)

The third possible command is used to send a robotaxi to another link in the network (rebalancing):

    void setRoboTaxiRebalance(Robotaxi, Request)



### Post-Execution Steps

With the first task, the designer can access the viewer for the final simulation run:

    void openViewer()

The next task is to retrieve the report and score information of the final simulation run:

    (html-report, score-information) = getFinalScore()

For more in-depth analysis, the simulation information can be downloaded from the server and processed locally.


## How to create your own submission {status=draft}

TODO: to write
