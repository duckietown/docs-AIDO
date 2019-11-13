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

## Docker {#cm-sw-docker}

Install Docker [from these instructions](https://docs.docker.com/install/).

## Git {#cm-sw-git}

We are sure you already have [set Git up](+software_reference#github-access).

## Duckietown Shell {#cm-sw-dts}

Install the Duckietown Shell by following the _Installation_ instructions
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

## [Optional] CUDA, cuDNN and Tensorflow {#cm-sw-tensorflow}

If you would like to use [Tensorflow](https://www.tensorflow.org/) to train your network and take advantage of your modern GPU, you should follow this short instruction to setup [CUDA](https://developer.nvidia.com/cuda-10.0-download-archive), [cuDNN](https://developer.nvidia.com/cudnn) and [Tensorflow-GPU](https://www.tensorflow.org/install/gpu)

Note that currently the baseline solution recommends Tensorflow-GPU 1.14, which requires CUDA 10.0 specifically. The latest CUDA is not 10.0 but rather 10.1. So you might need to downgrade your CUDA and Nvidia Driver in order to utilize GPU accelerated learning. **This tutorial assumes the user's system is 18.04**

It is always recommen to check first if your system's GPU support CUDA. You can do so by using:

    $ lspci | grep -i nvidia

### Clean previous Nvidia installation

Start installing by cleaning the ubuntu Nvidia install.

**Warning: This action might cause driver incompatibility and is only suggested for those who has some Ubuntu Experience.**

**In addition, it is recommen that a user SSH into the computer instead of using desktop UI as it might cause some extra issue with Nvidia Driver**

    $ sudo apt-get purge *nvidia*
    $ sudo apt-get purge *cuda*
    $ sudo rm /etc/apt/sources.list.d/cuda*
    $ sudo rm /etc/apt/sources.list.d/nvidia*
    $ sudo rm -rf /usr/local/cuda*
    $ sudo apt-get autoremove && sudo apt-get autoclean
    $ sudo apt-get update && sudo apt-get upgrade
    $ sudo add-apt-repository ppa:graphics-drivers/ppa

You should reboot your computer after these steps. Note that it is normal if your computer is not happy with its display driver. You might have to add `nomodeset` tag within your kernal temporarily to resolve kernal graphics issue. It is recommended to use CLI interface only to execute the following as utilizing graphics interface might cause unexpected issue. SSH is a good way of doing so.

At this point, your system should be prepped and all previous Nvidia dependencies has been eliminated. Now we can proceed to installing the necessary packages.

    $ sudo apt-get install g++ freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libglu1-mesa libglu1-mesa-dev

### Install CUDA 10.0

Download cuda deb installer from here:

    $ wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
    $ sudo dpkg -i cuda-repo-ubuntu1804_10.0.130-1_amd64.deb
    $ sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
    $ sudo apt-get update

**The following step is mandatory as it forces 10.0 installation:**

    $ sudo apt-get -o Dpkg::Options::="--force-overwrite" install cuda-10-0 cuda-drivers

CUDA path setup:

    $ echo 'export PATH=/usr/local/cuda-10.0/bin:$PATH' >> ~/.bashrc
    $ echo 'export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
    $ source ~/.bashrc
    $ sudo ldconfig

Once completed, you should restart your computer and at this point you can reuse your computer normally.

### Verify CUDA and Nvidia Driver install

You can verify CUDA install using the following command:

    $ nvcc --version

This will show as something like:

    Cuda compilation tools, release 10.0, V10.0.130

You can also verify that your system is utilizing GPU and see the GPU usage by using the following command:

    $ nvidia-smi

If you want to use this tool like `htop`, you can simply do:

    $ nvidia-smi -l 1

This will loop the readout and act like `htop` for GPU.

### Install cuDNN for CUDA 10.0

In order to download cuDNN you have to be regeistered [here](https://developer.nvidia.com/developer-program/signup) and then download cuDNN v7.5 form [here](https://developer.nvidia.com/cudnn)

**NOTE: you will need to select cuDNN v7.6.5 for CUDA 10.0**

Download all 3 Deb installer (Runtime Library, Developer Library, Code Samples and User Guide). You can install it by using command:

    $ sudo dpkg -i xxxxxxxxx.deb

Once complete, youshould also copy the file into cuda toolkit directory:

    $ sudo cp -P cuda/include/cudnn.h /usr/local/cuda-10.0/include
    $ sudo cp -P cuda/lib64/libcudnn* /usr/local/cuda-10.0/lib64/
    $ sudo chmod a+r /usr/local/cuda-10.0/lib64/libcudnn*

### Install Tensorflow-gpu

At this point, you have finished CUDA install, and cuDNN install, now its left is to install tensorflow gpu:

    $ pip3 install tensorflow-gpu==1.14

If you want to install system wide, you should do `sudo pip3 install`
