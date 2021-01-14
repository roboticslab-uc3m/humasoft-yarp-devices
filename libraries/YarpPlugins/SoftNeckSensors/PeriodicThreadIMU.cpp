#include "imudevice.hpp"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{    
    //Once imu is initilized, we get euler angles from it
    eulerdata = sensor->EulerAngles();

    //Publication of euler angles in Yarp
    Bottle data;
    data.clear();
    data.addDouble(eulerdata[0]);
    data.addDouble(eulerdata[1]);
    yarpPort.write(data);
    cout << "Writing bottle: " << data.toString() << endl;
}
