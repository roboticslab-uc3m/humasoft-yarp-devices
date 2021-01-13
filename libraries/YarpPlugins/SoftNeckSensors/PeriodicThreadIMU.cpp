#include "imudevice.hpp"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{    
    //Once imu is initilized, we get euler angles from it
    eulerdata = sensor->Euler_Angles();

    //Publication of euler angles in Yarp
    Bottle& data = yarpPort.prepare();
    data.addDouble(eulerdata[0]);
    data.addDouble(eulerdata[1]);
    yarpPort.write();
//    cout << "Writing bottle: \n" << data.toString() << endl;
    data.clear();    
}
