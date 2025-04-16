## SoftNeck usage instructions
You can use YARP module manager to simplify the execution of the different modules for soft-neck operation.
Other option is to run all of them in diferent terminals, in order to see the display of program output data.
In the next steps, you'll see the different modules that would have to be opened and the function it performs.

* **Yarp server**: This starts a name server running on the current machine
```bash
yarp server
```

### Sensors:

Choose the type of sensor you want to use according to the experiment:

* **Arduino IMU:** Its function is to collect the input data received by the Arduino and publish them on the yarp network.
```bash
yarpdev --device serialport --name /softimu --comport /dev/ttyACM0 --baudrate 9600 --paritymode NONE --databits 8 --stopbits 1
```
*Note*: For Arduino Nano, change `--comport / dev / ttyACM0` to `--comport / dev / ttyUSB0`

* **3DMGX510 IMU:**
```bash
sudo chmod 666 /dev/ttyUSB0 # To get permissions to access to /dev/ttyUSB0
yarpdev --device Imu --period 0.02
```

* **MOCAP:**
```bash
yarpdev --device Mocap --serverIP 2.2.2.103 --rigidBodyID 6
```

### Launch:

* **launchCanBus:** Start the iPOS, activating the motor control in position mode and open the necessary ports to control each of the 3 engines through the yarp network. `softNeck.ini` refers to the configuration of each iPOS
```bash
launchCanBus --from softNeck.ini
```

### Control module:

* **soft-neck-control in open-loop**: It will reach the commanded position starting from the initial position in which the neck is. Ideal for controlling with the SpaceMouse Wireless without using the IMU sensor.
   * Terminal 1: open loop control module
    ```bash
    yarpdev --device SoftNeckControl --name /SoftNeckControl --remoteRobot /softneck --fkPeriod 0 --coordRepr none --angleRepr polarAzimuth --angularUnits degrees
    ```
    * Terminal 2: to send commands
    ```bash
    yarp rpc /SoftNeckControl/rpc_transform:s
    > movj 20 10 # to move it in 20º inclination and 10º orientation
    ```
    Additionally you can connect this module with the **SpaceMouse Wireless**:
    * Terminal 3: to open new yarp ports where you are going to publish the mouse position outputs
    ```bash
    yarpdev --device SpaceNavigator --period 5 --name /spacenavigator --ports "(mouse buttons)" --channels 8 --mouse 0 5 0 5 --buttons 6 7 0 1
    ```
    * Terminal 4: module that connects the data output published by the mouse with the soft-neck-control in open loop
    ```bash
    streamingDeviceController --streamingDevice SpaceNavigator --remoteCartesian /SoftNeckControl --pose --gain 0.1 --SpaceNavigator::fixedAxes "(x y z rotz)" --period 0.01
    ```

* **soft-neck-control in closed-loop control**: It will reach the commanded position, using the IMU sensor to close the control loop
  - With **Sparkfun IMU** sensor:
      * Terminal 1: closed loop Coupled Control module using Sparkfun IMU (inclination-orientation)
      ```bash
      yarpdev --device SoftNeckControl --name /SoftNeckControl --remoteRobot /softneck --ImuSparkfun /softimu --coordRepr none --angleRepr polarAzimuth --angularUnits degrees --controlType ioCoupled --cmcPeriod 0.02
      ```
      * Terminal 1: closed loop Uncoupled Control module using Sparkfun IMU (inclination-orientation)
      ```bash
      yarpdev --device SoftNeckControl --name /SoftNeckControl --remoteRobot /softneck --ImuSparkfun /softimu --coordRepr none --angleRepr polarAzimuth --angularUnits degrees --controlType ioUncoupled --cmcPeriod 0.02
      ```
      * Terminal 2: to send commands to coupled or uncoupled control module using Sparkfun IMU (inclination-orientation)
      ```bash
      yarp rpc /SoftNeckControl/rpc_transform:s
      > stat        # to know the current IMU position
      > movj 20 10  # to move it in 20º inclination and 10º orientation
      ```
  - With **3DMGX510 IMU** sensor:
      * Terminal 1: closed loop Uncoupled Control module using 3DMGX510 IMU (roll-pitch)
      ```bash
      yarpdev --device SoftNeckControl --name /SoftNeckControl --remoteRobot /softneck --Imu3DMGX510 /softimu/out --coordRepr none --angleRepr polarAzimuth --angularUnits degrees --controlType rpUncoupled --cmcPeriod 0.02
      ```
      * Terminal 2: to send commands to Uncoupled control module using 3DMGX510 IMU (roll-pitch)
      ```bash
      yarp rpc /SoftNeckControl/rpc_transform:s
      > stat        # to know the current IMU position (roll pitch)
      > movj 20 10  # to move it in 20º inclination and 10º orientation
      ```
      * Terminal 3: you can check the differents [demostration programs](https://github.com/HUMASoft/yarp-devices/tree/develop/programs) to test the control and obtain system results.

  - With **MOCAP** system:
      * Terminal 1: closed loop Uncoupled Control module using MOCAP system (roll-pitch)
      ```bash
      yarpdev --device SoftNeckControl --name /SoftNeckControl --remoteRobot /softneck --Mocap /softmocap/out --coordRepr none --angleRepr polarAzimuth --angularUnits degrees --controlType rpUncoupled --cmcPeriod 0.02
      ```
      * Terminal 2: to send commands to Uncoupled control module using MOCAP system (roll-pitch)
      ```bash
      yarp rpc /SoftNeckControl/rpc_transform:s
      > stat        # to know the current MOCAP rigid body position (roll pitch)
      > movj 20 10  # to move it in 20º inclination and 10º orientation
      ```
