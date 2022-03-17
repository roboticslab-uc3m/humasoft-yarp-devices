#include "imudevice.hpp"

void IMUdevice::setupIMU() {

    //This method initializes the IMU, setting it to get euler angles at the desired frequency
    sensor = new IMU3DMGX510(comport, DEFAULT_FREQUENCY); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    if (sensor->check())
    {
        yarpPort.open(nameyarpoutport+"/out");
        cout << "Yarp port: " << nameyarpoutport+"/out" << " has been correctly opened." << endl;
    }
    sensor->set_freq(frequency);
    //Once the device is correctly connected, it's set to IDLE mode to stop transmitting data till user requests it
    sensor->set_streamon();

}

