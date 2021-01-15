#include "imudevice.hpp"

void IMUdevice::setupIMU() {

    //This method initializes the IMU, setting it to get euler angles at the desired frequency
    sensor = new IMU3DMGX510(comport); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    if (sensor->check()){
        yarpPort.open(nameyarpoutport);
        cout << "Yarp port: " << nameyarpoutport << " has been correctly opened." << endl;
    }
    sensor->set_freq(frequency);
    sensor->set_IDLEmode();
    sensor->set_devicetogetgyroacc();
    sensor->set_streamon();
    cout << "Calibrating IMU..." << endl;
    sensor->calibrate();
}
