# FhI Pino
FhI Pino documentation and examples

## Run the Robot

1. From a pilot PC, ssh to the FhI Pino Embedded machine: the two pc needs to be connected on the same network.

```
ssh pino@pino-embedded 
```

(You will need to use pino-embedded.local in some network)

Password will be communicated during the meeting.

pino-embedded has the following software features:

- Operating system: Ubuntu 20.04 with 5.10.89-xeno-ipipe-3.1 Xenomai RT Kernel
- Installed binary software: xbot2 suite for 20.04 + Xenomai
- Examples from sources: this repo

Note the following commands refer to the ssh session to the pino-embedded

2. Release the button, wait for 10 seconds and start the EtherCAT master 

```
ecat_master
```

3. If needed, regenerate the modular robot model

```
RobotBuilder
```

From the pilot PC open a web browser and connect to the pino-embedded machine on the port 5000

e.g. on the URL put: pino-embedded:5000 or pino-embedded.local:5000

NOTE: https is not supported at the moment

Click on the "Generate Model" button and after few seconds you will see the modular robot recognized and this should match the real one.

You can use th sliders on the top left corner to define a new homing configuration in joint space.

After this, click on the "Deploy Robot" button which will generate all the files needed to execute the modular robot.

4. set the xbot2 config file based on the generated modular robot that will be in the following path

```
set_xbot2_config ~/src/fhi_ws/src/ModularBot/config/ModularBot.yaml
```

NOTE: this need to be done only the first time we regenerate a modular robot model.

5. check that the config you set is the intended one and if needed tune some of the parameters

```
open_xbot2_config
```

6. start the xbot2 with the robot in impedance mode for example

```
xbot2-core --hw ec_imp
```

7. start the xbot2-gui on the pilot PC to check the status of the robot and to run a set of plugins.

```
xbot2-gui
```

## Pino examples

We prepared a set of examples to run:
- joint space control on the modular robot
- cartesian space control from RViZ using CartesI/O
- Gravity compensation example 

### Joint Space Control

Last time tutorial: https://github.com/ADVRHumanoids/modularbots_fraunhofer

### Cartesian space control

As an example we will use RViZ through the CartesI/O interactive markers to control the modular robot in the cartesian space.

To run it, on the pino-embedded:

```
mon launch ModularBot ModularBot_ik.launch
```

Then on the pilot PC:

```
rviz
```

### Gravity compensation example

On the xbot2 config file, by default, a *gcomp_example* plugin is provided.

To run it just start it from the xbot2-gui.

The source code can be found on this repo or on the ~/src/fhi_ws/src/pino_fhi/src/
