# Software requirements {#cm-sw status=ready}


This section describes the required software to participate in the competition.


## Supported platforms {#cm-sw-supported}


### Ubuntu 16 

Ubuntu 16 is the best supported environment.

### Ubuntu 18

Ubuntu 18 is the second best supported environment.

The reason it's not the first is that sometimes there 
is some confusion regarding Python 2 vs Python 3.

### Mac OS X

OS X is well supported; however we don't have full instructions
for certain steps. There is so much divergence in how OS X environments are configured.



## Docker  {#cm-sw-docker}

Install Docker [from these instructions](https://docs.docker.com/install/).

## Git {#cm-sw-git}

We are sure you already have Git, right?


## Duckietown Shell {#cm-sw-dts}


Install the Duckietown Shell using [these instructions](https://github.com/duckietown/duckietown-shell).


Make sure it's installed using:

    $ dts info
    
    
### Authentication token {#cm-sw-dts-token}

Set the Duckietown authentication token using this command:

    $ dts tok set
    
This checks that you have a good authentication token:

    $ dts challenges info


### Docker Hub information


Set your Docker Hub username using:

    $ dts challenges config --docker-username <USERNAME>

Login to Docker Hub:

    $ docker login
    
