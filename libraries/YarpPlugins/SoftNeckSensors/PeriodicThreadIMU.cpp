#include "imudevice.h"

// This method will be run in the background to get data from the IMU and publish it in Yarp

void IMUdevice::run()
{
    CD_INFO("Server Serial starting... \n");

    sensor = new IMU3DMGX510(comport); //Main key changing constructor of IMU3DMGX510 atribute string --> string portname = "..."
    sensor->set_IDLEmode();

    while (true) {

        sensor->set_devicetogetgyroacc(frequency);
        sensor->set_streamon();

        double *roll;
        double *pitch;
        double absrollaverage=0.0;
        double abspitchaverage=0.0;

        //Tal como está la función ahora, obtengo el número de muestras que le paso por parámetro
        std::tie( roll,  pitch,  absrollaverage,  abspitchaverage)= sensor->get_euleranglesContinuousStream(1000);
        sensor->set_streamoff();

        //Una vez las obtengo, las publico en Yarp, no puedo hacerlo uno a uno
        for (int i = 0 ; i<=899 ; i++){
        Bottle& data = yarpPort.prepare();
        data.addString("[");
        data.addDouble(*(roll+i));
        data.addString(",");
        data.addDouble(*(pitch+i));
        data.addString("]");
        yarpPort.write();
        cout << "Writing bottle: \n" << data.toString() << endl;
        data.clear();
        }

    }
    CD_INFO("Server Serial stopping... \n");
}

