# Software requirements {#cm-sw status=ready}


This section describes the required software to participate in the competition.

## Python {#cm-sw-python}

We require Python 3.6 or higher. For instructions for how to install Python 3.6 or higher, see [here](https://github.com/duckietown/duckietown-shell).


## Supported Operating Systems {#cm-sw-supported}


### Ubuntu 18

Ubuntu 18 is the best supported environment.


Note: In AI-DO 1, the best supported platform was Ubuntu 16.


### Mac OS X

OS X is well supported; however we don't have full instructions for certain steps.
(There is so much divergence in how OS X environments are configured.)




### Other operating systems

Any other OS with Python of at least version 3.6 should work. However,
we only support officially Ubuntu.


## Docker  {#cm-sw-docker}

Install Docker [from these instructions](https://docs.docker.com/install/).

## Git {#cm-sw-git}

We are sure you already have [set Git up](+software_reference#github-access).



## Duckietown Shell {#cm-sw-dts}


Install the Duckietown Shell by following the *Installation* instructions
in the [README](https://github.com/duckietown/duckietown-shell).


Make sure it is installed by using:

    $ dts version


### Authentication token {#cm-sw-dts-token}

Set the Duckietown authentication token using this command:

    $ dts tok set

This command checks that you have a good authentication token:

    $ dts challenges info


### Docker Hub information


Set your Docker Hub username using:

    $ dts challenges config --docker-username <USERNAME>

Login to Docker Hub:

    $ docker login
