#include "imudevice.hpp"

void IMUdevice::setupIMU() {

    //This method initializes the IMU, setting it to get euler angles at the desired frequency
    sensor = new IMU3DMGX510(comport); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    if (sensor->check()){
        yarpPort.open(nameyarpoutport+"/out");
        cout << "Yarp port: " << nameyarpoutport+"/out" << " has been correctly opened." << endl;
    }

    //IMU will be calibrated at DEFAULT_FREQUENCY
    sensor->set_freq(DEFAULT_FREQUENCY);
    sensor->set_IDLEmode();
    sensor->set_devicetogetgyroacc();
    sensor->set_streamon();
    cout << "Calibrating IMU..." << endl;
    sensor->calibrate();
    cout << "Calibration done" << endl;

    //Once it has been done, it's prepared to stream euler angles
    sensor->set_freq(frequency);
    sensor->set_IDLEmode();
    sensor->set_devicetogetgyroacc();
    sensor->set_streamon();

}

