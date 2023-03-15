#include "imudevice.hpp"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{
    if(output!="rp" && output!="py")
    {
        cout << "Error: wrong output option" << endl;
        return;
    }

    sensor->GetPitchRollYaw(pitch,roll,yaw);

    //Once imu is initilized, we get euler angles from it   
    if(output=="rp") //softneck output
    {
        // roll, pitch
        data.addFloat64(roll*180/M_PI);
        data.addFloat64(pitch*180/M_PI);
    }

    if(output=="py") //softarm output
    {
        // pitch, yaw       
        data.addFloat64( pitch*180/M_PI); // pitch in degrees
        data.addFloat64( yaw*180/M_PI); // yaw in degrees
    }

    yarpPort.write(data);
    cout << "Writing bottle: " << data.toString() << endl;
    data.clear();
}
