#include "imudevice.hpp"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{
    //Once imu is initilized, we get euler angles from it   
    if(output=="rp") //softneck output
    {
        eulerdata = sensor->EulerAngles();
        data.addDouble( eulerdata[0]);
        data.addDouble( eulerdata[1]);
    }

    if(output=="py") //softarm output
    {
        // pitch, yaw
        sensor->GetPitchRollYaw(eulerdata[0],eulerdata[1],eulerdata[2]);
        data.addDouble( eulerdata[0]*180/M_PI); // pitch in degrees
        data.addDouble( eulerdata[2]*180/M_PI); // yaw in degrees
    }

    yarpPort.write(data);
    cout << "Writing bottle: " << data.toString() << endl;
    data.clear();
}
