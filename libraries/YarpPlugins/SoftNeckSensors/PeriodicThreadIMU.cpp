#include "imudevice.h"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{
    CD_INFO("Server Serial starting... \n");

    sensor = new IMU3DMGX510(comport); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    sensor->set_IDLEmode();

    while (!PeriodicThread::isSuspended()) {

        sensor->set_devicetogetgyroacc(frequency);
        sensor->set_streamon();

        //Ahora mismo el sensor está emitiendo velocidades y aceleraciones angulares a la frecuencia indicada por el user
        eulerdata = sensor->Euler_Angles();
//        cout << "My attitude is (YX Euler): (" << eulerdata[0] << "," << eulerdata[1] << ")" << endl;

        Bottle& data = yarpPort.prepare();
        data.addString("[");
        data.addDouble(eulerdata[0]);
        data.addString(",");
        data.addDouble(eulerdata[1]);
        data.addString("]");
        yarpPort.write();
        cout << "Writing bottle: \n" << data.toString() << endl;
        data.clear();

    }
    CD_INFO("Server Serial stopping... \n");
}

