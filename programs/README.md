# Demostration Programs:
## 1. Step Input Demo:
Application designed to test the different types of control (coupled and uncoupled control) and the behavior of the system.
The application will move the neck in different poses, defined in this [lines](https://github.com/HUMASoft/yarp-devices/blob/18a6608dd6f5214b6bc5b02cac1e7ffd23541e01/programs/stepInputDemo/stepInputDemo.cpp#L79-L82) of the code:
```c++
    // Optional step inputs
    pose[0] = {15.0, 90.0};
    pose[1] = {20.0, 90.0};
    pose[2] = {20.0, 135.0};
```
Each pose will be executed in a [period](https://github.com/HUMASoft/yarp-devices/blob/18a6608dd6f5214b6bc5b02cac1e7ffd23541e01/programs/stepInputDemo/stepInputDemo.cpp#L84-L85) of 20 seconds
```c++
    // time/step (sec)
    double timeout = 20.0;
```
These parameters can be freely modified for each experiment.
In addition, the application can be run saving the table with the data regarding the reading of the sensors in a file `data.csv`. 
To do this, run the application with `csv` parameter:
```bash
stepInputDemo csv
```
### Results:

In this experiment the behavior between the coupled and uncoupled control system is compared:

<a href="https://vimeo.com/394899990" target="_blank"><img src="https://i.vimeocdn.com/video/860956792_640.jpg" 
alt="coupled vs uncoupled control" width="640" border="10" /></a>

Next you can visualize in jupyter-notebooks the dynamic graphs generated from the resulting files:
* [Step input using coupled control](https://nbviewer.jupyter.org/github/HUMASoft/Data-and-Results/blob/master/demo-results/jupyter-scripts/step-input-coupled-control.ipynb) ([csv](https://github.com/HUMASoft/Data-and-Results/blob/master/demo-results/csv-results/step-input-coupled-control-01.csv))
* [Step input using decoupled control](https://nbviewer.jupyter.org/github/HUMASoft/Data-and-Results/blob/master/demo-results/jupyter-scripts/step-input-decoupled-control.ipynb) ([csv](https://github.com/HUMASoft/Data-and-Results/blob/master/demo-results/csv-results/step-input-decoupled-control-01.csv))
* [Coupled control VS Decoupled control](https://nbviewer.jupyter.org/github/HUMASoft/Data-and-Results/blob/master/demo-results/jupyter-scripts/coupled-vs-decoupled-control.ipynb)

## 2. Joystick Input Demo:

Applications designed to test the different control systems, using the joystick position as the target position:
* **buttonPositionSend:** allows us to visualize the current inclination and orientation of the joystick on the screen and send it to the neck at the desired time, by pressing the side button on the joystick.
* **continuousPositionSend:** sends the joystick position to the control system continuously.

In continuousPositionSend, a `data.csv` file will be written, saving the joystick output values, the filtered joystick values (applying a low pass filter), and the sensor values.
To avoid strange behaviors in the sensor reading, the value to be commanded will start at 6 degree tilt and at 10 degree orientation.


### Results:

In this experiment you can see an example of the neck movement being commanded through the joystick, using the application `continuousPositionSend` in coupled control mode.

<a href="https://vimeo.com/399669993" target="_blank"><img src="https://i.vimeocdn.com/video/867802707_640.jpg" 
alt="joystick input using coupled control" width="640" border="10" /></a>

Next you can visualize in jupyter-notebook the dynamic graph generated from the resulting file:

* [Joystick input using coupled control](https://nbviewer.jupyter.org/github/HUMASoft/Data-and-Results/blob/master/demo-results/jupyter-scripts/joystick%20Input%20Coupled%20Control.ipynb) ([csv](https://github.com/HUMASoft/Data-and-Results/blob/master/demo-results/csv-results/joystick-input-coupled-control.csv))


## Data repository:
All the data captured for these experiments are stored in the repository: https://github.com/HUMASoft/Data-and-Results