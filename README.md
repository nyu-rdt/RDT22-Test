# RMC23-NUC
Contains NUC code for NYU RDTs 22-23 codebase

## State Machine Demo
This branch contains code to run a state machine demonstration using the RDT implementation of a state machine

### Usage 
This repository should be located in the src folder of a ROS2 workspace.

Build the workspace

You should see a package called "nyurdt_state_test" being created.

This package contains two executables:
> `single-machine`

> `nested-machine`

`single-machine` runs a simple linear state machine with 3 states.

`nested-machine` runs a nested state machine which houses 3 sub-machines.

To run each executable, use the `ros2 run` command like so:

> `ros2 run nyurdt_state_test single-machine`

> `ros2 run nyurdt_state_test nested-machine`