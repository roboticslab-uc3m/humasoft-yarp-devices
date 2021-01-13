#include "imu3dmgx510.h"


IMU3DMGX510::IMU3DMGX510(string portName) : port(portName) {

    //3DMGX10 device will be calibrated once port where it has been connected to has been correctly opened thanks to SerialComm constructor.
    estimador.setMagCalib(0.0, 0.0, 0.0); //Device 3DMGX10 has no magnetometer
    estimador.setGyroBias(bx,by,bz); //Setting of gyro bias
    estimador.setPIGains(Kp, Ti, KpQuick, TiQuick); //Setting of device gains
}


int IMU3DMGX510::start() {
    port.WriteLine(idle);
    int check = port.CheckLine(respuestacorrectaidle);
    return check;
}

int IMU3DMGX510::set_IDLEmode() {

    //We send data to set 3DMGX10 to IDLE mode
    port.WriteLine(idle);

    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    int comprobacion = port.CheckLine(respuestacorrectaidle);

    return comprobacion;
}

long IMU3DMGX510::set_streamon(){

    //We activate data stream
        port.WriteLine(streamon);

        //3DMGX10 will answer back a message showing if any error appeared in the process
        //We must read it
        int comprobacion = port.CheckLine(respuestacorrectastreamonoff);
        if (comprobacion == 1){
//            cout << "Envio y respuesta correctos" << endl;
        }
    return 0;
}

long IMU3DMGX510::set_streamoff(){

    //We activate data stream
        port.WriteLine(streamoff);

        //3DMGX10 will answer back a message showing if any error appeared in the process
        //We must read it
        int comprobacion = port.CheckLine(respuestacorrectastreamonoff);
        if (comprobacion == 1){
//            cout << "Envio y respuesta correctos" << endl;
        }
    return 0;
}

long IMU3DMGX510::set_reset(){

    //We activate data stream
        port.WriteLine(reset);

        //3DMGX10 will answer back a message showing if any error appeared in the process
        //We must read it
        int comprobacion = port.CheckLine(respuestacorrectareset);
        if (comprobacion == 1){
//            cout << "Envio y respuesta correctos" << endl;
        }
    return 0;
}

long IMU3DMGX510::set_devicetogetgyroacc(int freq){

    //We will prepare our device to get gyros and accs values
    //Freq will be introduced by user (1Hz or 100Hz atm)

    if (freq==1){
        port.WriteLine(gyracc);

    }else if(freq==100){
        port.WriteLine(gyracc100);
    }

    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    int comprobacion = port.CheckLine(respuestacorrectaajustes);
    if (comprobacion == 1){
//        cout << "Envio y respuesta correctos" << endl;
    }
    return 0;

}

long IMU3DMGX510::set_devicetogetgyro(int freq){

    //We will prepare our device to get gyros and accs values
    //Freq will be introduced by user (1Hz or 100Hz atm)
    if (freq==1){
        port.WriteLine(imudata1);
    }else if (freq==100){
        port.WriteLine(imudata100);
    }else if (freq==1000){
        port.WriteLine(imudata1000);
    }

    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    int comprobacion = port.CheckLine(respuestacorrectaajustes);
    if (comprobacion == 1){
//        cout << "Envio y respuesta correctos" << endl;
    }
    return 0;

}



std::tuple <float, float, float> IMU3DMGX510::get_gyroPolling() {

    //We send data to set 3DMGX10 to polling mode
    port.WriteLine(polling);

    string reading;
    float gyroxvalue, gyroyvalue, gyrozvalue;

    //3DMGX10 will answer back a message with gyro values
    //We must read it
    reading = port.ReadNumberofChars(20);


        //X
        ulf x;
        reading.substr(6,4);
        std::string str =hex(reading.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> x.ul;
        gyroxvalue = x.f;


        //y
        ulf y;
        std::string str1 =hex(reading.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> y.ul;
        gyroyvalue = y.f;


        //z
        ulf z;
        std::string str2 =hex(reading.substr(14,4));
        std::stringstream ss2(str2);
        ss2 >> std::hex >> z.ul;
        gyrozvalue = z.f;

    cout << "Our gyro velocities are: "<< gyroxvalue << " " << gyroyvalue << " " << gyrozvalue << endl;


    return std::make_tuple(gyroxvalue, gyroyvalue, gyrozvalue);

}
double* IMU3DMGX510::get_euleranglesPolling() {


    string reading;
    double roll, pitch;
    static double estimation[2];
    char c;
    int comp=0;
    int fin=0;
    int firsttime;

    //First time needs a mayor number of samples in order to get calibrated
    //After it, there is no need of taking such number of samples

    if (firsttime==0){

    for (int i =0; i<=100 ; i++){

        firsttime=1;


        //We send data to set 3DMGX10 to polling mode
        port.WriteLine(polling);

        //3DMGX10 will answer back a message with gyro and acc values
        //We must read it
        //reading = port.ReadNumberofChars(34);
        //cout << "Lo que leemos es: " << hex(reading) << endl;

        // 7565 80 1C 0E 04 3DA807A0 3EC36059 3F696C54 0E 05 BE6745FE 3E17130F BDCEB19C 00C2


        comp=0;
        fin=0;

        do{
            c = port.ReadChar();
            switch (c) {
            case 'u':{
                comp=1;
                reading+=c;
                break;}
            case 'e':{
                if (comp==1){
                    fin=1;
                    reading+=c;
                }else{
                    comp=0;
                    reading+=c;
                }
                break;}

            default:{
                reading+=c;
                break;}

            }
        }while(fin==0);

        char descriptor = port.ReadChar();
        reading+=descriptor;

        char longitud = port.ReadChar();
        reading+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            c = port.ReadChar();
            reading+=c;
        }

        if (int(longitud) == 28){
            ulf accx;
            std::string str =hex(reading.substr(6,4));
            std::stringstream ss(str);
            ss >> std::hex >> accx.ul;
            double f = accx.f;

            ulf accy;
            std::string str1 =hex(reading.substr(10,4));
            std::stringstream ss1(str1);
            ss1 >> std::hex >> accy.ul;
            double f1 = accy.f;

            ulf accz;
            std::string str2 =hex(reading.substr(14,4));
            std::stringstream ss2(str2);
            ss2 >> std::hex >> accz.ul;
            double f2 = accz.f;

            ulf gyrox;
            std::string str3 =hex(reading.substr(20,4));
            std::stringstream ss3(str3);
            ss3 >> std::hex >> gyrox.ul;
            double f3 = gyrox.f;

            ulf gyroy;
            std::string str4 =hex(reading.substr(24,4));
            std::stringstream ss4(str4);
            ss4 >> std::hex >> gyroy.ul;
            double f4 = gyroy.f;

            ulf gyroz;
            std::string str5 =hex(reading.substr(28,4));
            std::stringstream ss5(str5);
            ss5>> std::hex >> gyroz.ul;
            double f5 = gyroz.f;

            estimador.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);

        }
    }
    }else{

        //We send data to set 3DMGX10 to polling mode
                port.WriteLine(polling);

                //3DMGX10 will answer back a message with gyro and acc values
                //We must read it
                //reading = port.ReadNumberofChars(34);
                //cout << "Lo que leemos es: " << hex(reading) << endl;

                // 7565 80 1C 0E 04 3DA807A0 3EC36059 3F696C54 0E 05 BE6745FE 3E17130F BDCEB19C 00C2


                comp=0;
                fin=0;

                do{
                    c = port.ReadChar();
                    switch (c) {
                    case 'u':{
                        comp=1;
                        reading+=c;
                        break;}
                    case 'e':{
                        if (comp==1){
                            fin=1;
                            reading+=c;
                        }else{
                            comp=0;
                            reading+=c;
                        }
                        break;}

                    default:{
                        reading+=c;
                        break;}

                    }
                }while(fin==0);

                char descriptor = port.ReadChar();
                reading+=descriptor;

                char longitud = port.ReadChar();
                reading+=longitud;

                for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
                    c = port.ReadChar();
                    reading+=c;
                }

                if (int(longitud) == 28){
                    ulf accx;
                    std::string str =hex(reading.substr(6,4));
                    std::stringstream ss(str);
                    ss >> std::hex >> accx.ul;
                    double f = accx.f;

                    ulf accy;
                    std::string str1 =hex(reading.substr(10,4));
                    std::stringstream ss1(str1);
                    ss1 >> std::hex >> accy.ul;
                    double f1 = accy.f;

                    ulf accz;
                    std::string str2 =hex(reading.substr(14,4));
                    std::stringstream ss2(str2);
                    ss2 >> std::hex >> accz.ul;
                    double f2 = accz.f;

                    ulf gyrox;
                    std::string str3 =hex(reading.substr(20,4));
                    std::stringstream ss3(str3);
                    ss3 >> std::hex >> gyrox.ul;
                    double f3 = gyrox.f;

                    ulf gyroy;
                    std::string str4 =hex(reading.substr(24,4));
                    std::stringstream ss4(str4);
                    ss4 >> std::hex >> gyroy.ul;
                    double f4 = gyroy.f;

                    ulf gyroz;
                    std::string str5 =hex(reading.substr(28,4));
                    std::stringstream ss5(str5);
                    ss5>> std::hex >> gyroz.ul;
                    double f5 = gyroz.f;

                    estimador.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);

                }

    }
    estimation[0]=estimador.eulerRoll();
    estimation[1]=estimador.eulerPitch();
    //cout << "(" << estimation[0] << "," << estimation[1] << ")" << endl;

    return estimation;
}

std::tuple <double*,double*,double*> IMU3DMGX510::get_gyroContinuousStream(int samples){

    //Decl. of the variables
     char c;
     string answer;
     char descriptor;
     char longitud;
     static double gyroxvector[10000];
     static double gyroyvector[10000];
     static double gyrozvector[10000];

     //The methodology will be the next:
     //   1) First, we will go through the 1st do-while loop untill we find 'ue' in our data packet. Then, varible "fin" will be set to 1.
     //   2) After reading 'ue', two next bites in the data packet are the descriptor and the lenght. Both will be read.
     //   3) We read the entire data packet with a for loop and its limit set by the lenght of the packet.
     //   4) We extract gyro values (gyrx,gyry,gyrz) from the recent read data packet.
     //   5) Repeat all steps "muestras" times.

     for (int h=0; h<=samples;h++){

         //Reset of some variables to avoid infinite loops
         int comp=0;
         int fin=0;

         do{
             c = port.ReadChar();
             switch (c) {
             case 'u':{
                 comp=1;
                 answer+=c;
                 break;}
             case 'e':{
                 if (comp==1){
                     fin=1;
                     answer+=c;
                 }else{
                     comp=0;
                     answer+=c;
                 }
                 break;}

             default:{
                 break;}
             }
         }while(fin==0);

         descriptor = port.ReadChar();
         answer+=descriptor;

         longitud = port.ReadChar();
         answer+=longitud;

         for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
             c = port.ReadChar();
             answer+=c;
         }

         if (int(longitud) == 14){ //It must be 14
             //X

             ulf x;
             std::string str =hex(answer.substr(6,4));
             std::stringstream ss(str);
             ss >> std::hex >> x.ul;
             float f = x.f;
             gyroxvector[h] = f;

             //y

             ulf y;
             std::string str1 =hex(answer.substr(10,4));
             std::stringstream ss1(str1);
             ss1 >> std::hex >> y.ul;
             float f1 = y.f;
             gyroyvector[h] = f;

             //z

             ulf z;
             std::string str2 =hex(answer.substr(14,4));
             std::stringstream ss2(str2);
             ss2 >> std::hex >> z.ul;
             float f2 = z.f;
             gyrozvector[h] = f;
             }

         answer.clear();

         }

     return std::make_tuple(gyroxvector, gyroyvector, gyrozvector);
}
std::tuple <double*,double*,double,double> IMU3DMGX510::get_euleranglesContinuousStream(int samples){



//    //We need to configurate the device to get gyros and accs
//    port.WriteLine(gyracc100);

//    //3DMGX10 will answer back a message showing if any error appeared in the process
//    //We must read it

//    int comprobacion = port.CheckLine(respuestacorrectaajustes);
//    if (comprobacion == 1){
//        cout << "Envio y respuesta correctos" << endl;
//    }

//    //Beginning of the stream
//    port.WriteLine(streamon);

//    int comprobacion2 = port.CheckLine(respuestacorrectastreamonoff);
//    if (comprobacion2 == 1){
//        cout << "Envio y respuesta correctos" << endl;
//    }

    //Decl. of the variables
    string answer;
    char c;
    char longitud;
    char descriptor;
    static double rollvector[10000];
    static double pitchvector[10000];
    double rollaverage=0.0;
    double pitchaverage=0.0;

    //        The methodology will be the next:
    //           1) First, we will go through the 1st do-while loop untill we find 'ue' in our data packet. Then, varible "fin" will be set to 1.
    //           2) After reading 'ue', two next bites in the data packet are the descriptor and the lenght. Both will be read.
    //           3) We read the entire data packet with a for loop and its limit set by the lenght of the packet.
    //           4) We extract float gyro values (gyrox,gyroy,gyroz) and float acc values(accx,accy,accz) from the recent read data packet.
    //           5) Now, we need to converts these values to Euler Angles(Pitch,Roll). "Attitude_estimator" lib is used to perform it.
    //           6) Once the receiving values are stable, we use the comment library to get Pitch,Roll. (If device is placed face down, values are stable since the very beginning).
    //           7) To correct the initial offset, we will get the average value of the first 125 values. This way, if our initial offset Yaw it 2'5, a correct value to the measurings of this angle will be: measuring - 2'5.
    //           8) Repeat all steps "muestras" times.

        for (int h=0; h<=samples;h++){

            //Reset of the variables to avoid infinite loops.
             answer.clear();
             int comp=0;
             int fin=0;

            do{
                c = port.ReadChar();
                switch (c) {
                case 'u':{
                    comp=1;
                    answer+=c;
                    break;}
                case 'e':{
                    if (comp==1){
                        fin=1;
                        answer+=c;
                    }else{
                        comp=0;
                        answer+=c;
                    }
                    break;}

                default:{
                    answer+=c;
                    break;}

                }
            }while(fin==0);

            descriptor = port.ReadChar();
            answer+=descriptor;

            longitud = port.ReadChar();
            answer+=longitud;

            for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
                c = port.ReadChar();
                answer+=c;
            }

            if (int(longitud) == 28){
            ulf accx;
            std::string str =hex(answer.substr(6,4));
            std::stringstream ss(str);
            ss >> std::hex >> accx.ul;
            double f = accx.f;

            ulf accy;
            std::string str1 =hex(answer.substr(10,4));
            std::stringstream ss1(str1);
            ss1 >> std::hex >> accy.ul;
            double f1 = accy.f;

            ulf accz;
            std::string str2 =hex(answer.substr(14,4));
            std::stringstream ss2(str2);
            ss2 >> std::hex >> accz.ul;
            double f2 = accz.f;

            ulf gyrox;
            std::string str3 =hex(answer.substr(20,4));
            std::stringstream ss3(str3);
            ss3 >> std::hex >> gyrox.ul;
            double f3 = gyrox.f;

            ulf gyroy;
            std::string str4 =hex(answer.substr(24,4));
            std::stringstream ss4(str4);
            ss4 >> std::hex >> gyroy.ul;
            double f4 = gyroy.f;

            ulf gyroz;
            std::string str5 =hex(answer.substr(28,4));
            std::stringstream ss5(str5);
            ss5>> std::hex >> gyroz.ul;
            double f5 = gyroz.f;


            //If sensor is placed face down, we can skip the if loop.
            if (h>=100 && h<=samples){

                estimador.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
                rollvector[h-100]=estimador.eulerRoll();
                pitchvector[h-100]=estimador.eulerPitch();
                cout << "My attitude is (YX Euler): (" << estimador.eulerPitch() << "," << estimador.eulerRoll() << ")" << endl;


                if(h>=225 && h<=350){

                    rollaverage= rollaverage + estimador.eulerRoll();
                    pitchaverage= pitchaverage + estimador.eulerPitch();

                    if (h==350){

                        rollaverage = rollaverage/125;
                        pitchaverage = pitchaverage/125;
                    }
                }
            }
           }
        }



        return std::make_tuple(rollvector,pitchvector, rollaverage, pitchaverage);
    }

double* IMU3DMGX510::Euler_Angles() {

    string answer;
    char c;
    char longitud;
    char descriptor;

    static double EulerAngles[2];


    //Reset of the variables to avoid infinite loops.
    answer.clear();
    int comp=0;
    int fin=0;

    do{
        c = port.ReadChar();
        switch (c) {
        case 'u':{
            comp=1;
            answer+=c;
            break;}
        case 'e':{
            if (comp==1){
                fin=1;
                answer+=c;
            }else{
                comp=0;
                answer+=c;
            }
            break;}

        default:{
            answer+=c;
            break;}

        }
    }while(fin==0);

    descriptor = port.ReadChar();
    answer+=descriptor;

    longitud = port.ReadChar();
    answer+=longitud;

    for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
        c = port.ReadChar();
        answer+=c;
    }

    if (int(longitud) == 28){
        ulf accx;
        std::string str =hex(answer.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> accx.ul;
        double f = accx.f;

        ulf accy;
        std::string str1 =hex(answer.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> accy.ul;
        double f1 = accy.f;

        ulf accz;
        std::string str2 =hex(answer.substr(14,4));
        std::stringstream ss2(str2);
        ss2 >> std::hex >> accz.ul;
        double f2 = accz.f;

        ulf gyrox;
        std::string str3 =hex(answer.substr(20,4));
        std::stringstream ss3(str3);
        ss3 >> std::hex >> gyrox.ul;
        double f3 = gyrox.f;

        ulf gyroy;
        std::string str4 =hex(answer.substr(24,4));
        std::stringstream ss4(str4);
        ss4 >> std::hex >> gyroy.ul;
        double f4 = gyroy.f;

        ulf gyroz;
        std::string str5 =hex(answer.substr(28,4));
        std::stringstream ss5(str5);
        ss5>> std::hex >> gyroz.ul;
        double f5 = gyroz.f;

        estimador.update(0.01,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
        EulerAngles[0]=estimador.eulerRoll();
        EulerAngles[1]=estimador.eulerPitch();

        }
//        cout << "My attitude is (YX Euler): (" << estimador.eulerPitch() << "," << estimador.eulerRoll() << ")" << endl;

    return EulerAngles;
}
