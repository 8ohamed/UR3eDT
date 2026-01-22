# Pre-requisites

This folder contains the pre-requisites for getting started with the project.

The pre-requisites are indexed in the way that they should be followed. The pre-requisites are:

   1. [0_environment_setup](0_environment_setup.ipynb): This folder contains the initial setup for the project. The initial setup is to install the necessary packages for the project.

## Additional obligatory pre-requisites

Some Windows users might encounter issues when installing packages if they do not have the required build tools installed. If you run into errors related to missing C++ build tools during package installation, please ensure you have the Microsoft Visual C++ Build Tools installed on your system (you can follow the solution on this [Stack Overflow post](https://stackoverflow.com/a/64262038)).

As some of the material in this repository is based on the material found in the [incubator case study](https://github.com/clagms/IncubatorDTCourse) by Cláudio Gomes, the following pre-requisites from that repository are also obligatory:

   1. [Docker](https://github.com/clagms/IncubatorDTCourse/blob/main/0-Pre-requisites/2-Docker.ipynb): Used to run the RabbitMQ image.
   2. [RabbitMQ](https://github.com/clagms/IncubatorDTCourse/blob/main/0-Pre-requisites/3-RabbitMQ.ipynb): Used for exchanging messages between the different components of the system.
