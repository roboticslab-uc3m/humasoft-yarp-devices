## SoftArm usage instructions
You can use YARP module manager to simplify the execution of the different modules for soft-neck operation.
Other option is to run all of them in diferent terminals, in order to see the display of program output data.
In the next steps, you'll see the different modules that would have to be opened and the function it performs.

* **Yarp server**: This starts a name server running on the current machine
```bash
yarp server
```

### Sensors:

* **3DMGX510 IMU:**
```bash
sudo yarpdev --device Imu --output py --freq 50
```

### Launch:

* **launchCanBus:** Start the iPOS, activating the motor control in position mode and open the necessary ports to control each of the 3 engines through the yarp network. `softArm.ini` refers to the configuration of each iPOS
```bash
launchCanBus --from softArm.ini
```

### Control module:

* **soft-arm-control in open-loop**: It will reach the commanded position starting from the initial position in which the neck is. Note: in open loop, the system will solve the inverse kinematic using mathematical method by default. If you want, the system can take the IK result using a table adding the parameter: --tableIk *pathFile*. For example, you can add [Table170.csv](https://drive.google.com/file/d/11WGYk2OSIJfw9gZr_Mzdz_D2S0rOdJqH/view?usp=sharing) writing --tableIk ~/Table170.csv if you've located this file in your home dir.
   * Terminal 1: open loop control module
    ```bash
    yarpdev --device SoftArmControl --name /SoftArmControl --remoteRobot /softarm --fkPeriod 0 --coordRepr none --angleRepr polarAzimuth --angularUnits degrees
    ```
    * Terminal 2: to send commands
    ```bash
    yarp rpc /SoftArmControl/rpc_transform:s
    > movj 20 10 # to move it in 20º inclination and 10º orientation
    ```
    This is deal for controlling with the SpaceMouse Wireless without using the IMU sensor. You can connect this module with the **SpaceMouse Wireless**:
    * Terminal 3: to open new yarp ports where you are going to publish the mouse position outputs
    ```bash
    yarpdev --device SpaceNavigator --period 5 --name /spacenavigator --ports "(mouse buttons)" --channels 8 --mouse 0 5 0 5 --buttons 6 7 0 1
    ```
    * Terminal 4: module that connects the data output published by the mouse with the soft-arm-control in open loop
    ```bash
    streamingDeviceController --streamingDevice SpaceNavigator --remoteCartesian /SoftArmControl --pose --gain 0.03 --SpaceNavigator::fixedAxes "(x y z rotz)" --period 0.02
    ```

* **soft-arm-control in closed-loop control**: It will reach the commanded position, using the IMU sensor to close the control loop. Note: internally, both the sensor and the control loop, work with pitch and yaw. Instead, the user will always enter *inclination and orientation* values into the system to indicate the position of the arm.
  - With **3DMGX510 IMU** sensor:
      * Terminal 1: closed loop Uncoupled Control module using 3DMGX510 IMU (pitch-yaw)
      ```bash
      yarpdev --device SoftArmControl --name /SoftArmControl --remoteRobot /softarm --remoteSensor /softimu/sensor:o --coordRepr none --angleRepr polarAzimuth --angularUnits degrees --cmcPeriod 0.02
      ```
      * Terminal 2: to send commands to Uncoupled control module using 3DMGX510 IMU (pitch-yaw)
      ```bash
      yarp rpc /SoftArmControl/rpc_transform:s
      > stat        # to know the current IMU position (inclination orientation)
      > movj 20 90  # to move it in 20º inclination and 10º orientation
      ```
