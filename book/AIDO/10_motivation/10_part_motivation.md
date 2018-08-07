# Introduction {#part:aido-introduction status=beta}

Maintainer: Julian Zilly

We introduce the *AI Driving Olympics* (AIDO), its relevance, environment and tasks. This competition is aimed as a stepping stone to understand the role of AI in robotics in general and in self-driving cars in particular. For more on the motivation and reasoning behind the competition see our ["announcement paper"](https://drive.google.com/file/d/1bRERCWWt2k-zGO0f5kB8pDRAuRCrZ4Wv/view) and [AIDO overview](#aido-overview).


The *AI Driving Olympics* comprise a set of four tasks each of which highlights a certain part of robotic driving. Tasks are evaluated both on how fast they drive as well as their traffic law abidance and comfort of driving. Furthermore we offer a "purist" (P) and "non-purist" (NP) way of participating depending on whether computation is performed on the RaspberryPi or uses additional computational resources. The following presents the description, performance and evaluation of tasks separately. Please follow the linked titles for further information.


**Main links**

* For an overview of the tasks, please see: [**Task overview**](#task_overview).

* For a description of the performance objectives of each task, please see: [**Rules**](#aido-rules).

* Where to get started with your code submission: [**User submission**](#aido-quickstart)



In total the following ways of participating are offered. There are different tasks, different computational resource regimes and different performance categories in which to compete.

### Tasks

* [Lane following (LF)](#lf)
* [Lane following with dynamic vehicles (LFV)](#lf_v)      
* [Navigation with dynamic vehicles (NAVV)](#nav_v)
* [Autonomous mobility-on-demand (AMOD)](#amod)

### Computational resources

* [Purist option - RaspberryPi](#computation)
* [Non-purist option - Additional Movidius stick](#computation)


### Performance metrics

* [Performance](#performance)
* [Traffic law abidance](#traffic_laws)
* [Comfort](#comfort)

-----------------------------------------

**Important remarks**

The rules and descriptions in this document are subject to change at the discretion of the authors. In particular we aim to avoid cases where due to "loop holes" the best solution to a task ends up breaking with the spirit of the task, e.g. it should not be favorable to drive off road to get to a destination faster. Likewise if circumstances in the AIDO software and hardware require adjustments to the rules these will be taken. Having clarified this, the foremost goal is not to change the rules unless necessary.

We invite discussion of the rules and evaluation by participants. At the bottom of each webpage we provide a link to ask questions and suggest improvements.
