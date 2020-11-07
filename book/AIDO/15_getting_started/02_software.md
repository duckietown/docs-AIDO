# Software requirements {#cm-sw status=ready}

This section describes the required software to participate in the competition.
<!-- 
## Python {#cm-sw-python}

We require Python 3.8 or higher.  -->

<!-- For instructions for how to install Python 3.8 or higher, see [here](https://github.com/duckietown/duckietown-shell). -->

## Supported Operating Systems {#cm-sw-supported}

### Ubuntu 20.04

Ubuntu 20.04 is the best supported environment.
Earlier version might work. Note that we require an environment with Python 3.8 or higher.

### Other GNU/Linux versions

Any other GNU/Linux OS with Python of at least version 3.8 should work. However,
we only support officially Ubuntu.

### Mac OS X

OS X is well supported; however we don't have full instructions for certain steps.
(There is so much divergence in how OS X environments are configured.)

We suggest to use `pyenv` to install Python 3.8.

### Windows

Windows is not supported.



## Docker  {#cm-sw-docker}

Install Docker [from these instructions](https://docs.docker.com/install/).

Note: [Mac users] Unfortunately Docker Desktop 2.4 and 2.5
have known issues that prevent it to work with AI-DO.
We suggest that you install Docker Desktop 2.3.x.

If you want to use a GPU for evaluating your submission,
make sure to edit your `/etc/docker/daemon.json` to include
the following options.

```json
{
    "default-runtime": "nvidia",

    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },

    "node-generic-resources": [ "NVIDIA-GPU=0" ]
}
```

Note: you likely know about the first two options `default-runtime`
and `runtimes`. Be sure to include also the "unusual"
option `node-generic-resources`: this is needed because the evaluation
uses Docker Compose.

## Git {#cm-sw-git}

We need Git and Git LFS.

## Duckietown Shell {#cm-sw-dts}

Install the [Duckietown Shell][shell] using:

    $ pip install -U duckietown-shell

If you encounter problems look at *Installation* instructions
in the [README][shell].

[shell]: https://github.com/duckietown/duckietown-shell

Make sure it is installed by using:

    $ dts version


<img class="screencast" src="rec-dts-version.gif" width="100%"/>

Set the `daffy` command branch:

    $ dts --set-version daffy exit

<img class="screencast" src="rec-dts-command-version.gif" width="100%"/>

Update the commands using:

    $ dts update

<style>
@media print {
  .screencast {
    display: none;
  }
}

</style>

<!-- <img src="rec-dts-update.gif" width="70%"/> -->

### Authentication token {#cm-sw-dts-token}

Set the Duckietown authentication token using this command:

    $ dts tok set

### Docker Hub information

Set your Docker Hub username and password using:

    $ dts challenges config --docker-username <USERNAME> --docker-username <PASSWORD>

You can use an access token instead of a password.

Login to Docker Hub:

    $ docker login


Note: Since November 2, 2020 Docker Hub has implemented tight
rate limits for anonymous accounts. If you experience
timeouts in Docker or similar problems, it is likely 
because you have not logged in recently. Note that `docker login`
needs to be repeated every 12 hours.

### Check `dts` configuration

This command checks that you have a good authentication token:

    $ dts challenges info

<img src="rec-challenges-info.gif" style='width: 80%'/>

You should expect an output like:

```
~        You are succesfully authenticated:
~
~                     ID: ![your numeric ID]
~                   name: ![your name]
~                  login: ![your account name on Duckietown]
~                profile: ![your website]
~
~            You can find the list of your submissions at the page:
~
~                https://challenges.duckietown.org/v4/humans/users/1639
```

