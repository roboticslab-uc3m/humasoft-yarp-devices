#include "imu3dmgx510.h"


// -------------------------  Constructor  -------------------------------

IMU3DMGX510::IMU3DMGX510(string portName) : port(portName) {
    //3DMGX10 device will be calibrated once port where it has been connected to has been correctly opened thanks to SerialComm constructor.
    estimador.setMagCalib(0.0, 0.0, 0.0); //Device 3DMGX10 has no magnetometer
    estimador.setGyroBias(bx,by,bz); //Setting of gyro bias
    estimador.setPIGains(Kp, Ti, KpQuick, TiQuick); //Setting of device gains
}

// -----------------------------------------------------------------------

// -------------------------  Initialization  ----------------------------

bool IMU3DMGX510::check() {
    bool check;
    port.WriteLine(idle);
    do{
    check = port.CheckLine(respuestacorrectaidle,idle);
    }while (check==false);
    return check;
}

bool IMU3DMGX510::set_freq(int frequency){
    freq=frequency;
    period = 1 / freq;
    return true;
}

bool IMU3DMGX510::calibrate(){

    //We will obtain initial offset to correct it the moment we make measures
    string answer, str, str1, str2, str3, str4, str5;
    char c;
    char longitud;
    char descriptor;
    double roll=0.0;
    double pitch=0.0;
    ulf accx,accy,accz,gyrox,gyroy,gyroz;

    for (int h=0; h<=500;h++){

        //Reset of the variables to avoid infinite loops.
        answer.clear();
        str.clear();
        str1.clear();
        str2.clear();
        str3.clear();
        str4.clear();
        str5.clear();
        int comp = 0;
        int fin = 0;
        double f=0;
        double f1=0;
        double f2=0;
        double f3=0;
        double f4=0;
        double f5=0;
        c = '0';
        longitud = '0';
        descriptor = '0';

        do{
            c = port.GetChar();
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

        descriptor = port.GetChar();
        answer+=descriptor;

        longitud = port.GetChar();
        answer+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            c = port.GetChar();
            answer+=c;
        }

        if (int(longitud) == 28 && int(descriptor) == -128){
            str =hex(answer.substr(6,4));
            std::stringstream ss(str);
            ss >> std::hex >> accx.ul;
            f = accx.f;

            str1 =hex(answer.substr(10,4));
            std::stringstream ss1(str1);
            ss1 >> std::hex >> accy.ul;
            f1 = accy.f;

            str2 =hex(answer.substr(14,4));
            std::stringstream ss2(str2);
            ss2 >> std::hex >> accz.ul;
            f2 = accz.f;

            str3 =hex(answer.substr(20,4));
            std::stringstream ss3(str3);
            ss3 >> std::hex >> gyrox.ul;
            f3 = gyrox.f;

            str4 =hex(answer.substr(24,4));
            std::stringstream ss4(str4);
            ss4 >> std::hex >> gyroy.ul;
            f4 = gyroy.f;

            str5 =hex(answer.substr(28,4));
            std::stringstream ss5(str5);
            ss5>> std::hex >> gyroz.ul;
            f5 = gyroz.f;

            estimador.update(period,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
            roll = estimador.eulerRoll();
            pitch= estimador.eulerPitch();

            //First 150 measures are ignored because they are not stable
            if (h>=150 && h<=500){
                rolloffset= rolloffset+ roll ;
                pitchoffset= pitchoffset +pitch ;
            }
        }
    }
    rolloffset = rolloffset / 350;
    pitchoffset = pitchoffset / 350;

//    rolloffset=rolloffset*180/M_PI;
//    pitchoffset=pitchoffset*180/M_PI;
    cout << "Initial offsets: \n" << "Roll = " << rolloffset << "\n" << "Pitch = " << pitchoffset << endl;
    return true;
}

// -----------------------------------------------------------------------

// -------------------------  Configuration  -----------------------------

bool IMU3DMGX510::set_IDLEmode() {
    bool comprobacion;
    //We send data to set 3DMGX10 to IDLE mode
    port.WriteLine(idle);
    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    //Included loop to restart the process if it's frozen. It happens sometimes 1st time imu is active
    do{
    comprobacion = port.CheckLine(respuestacorrectaidle,idle);
    }while (comprobacion==false);
    return comprobacion;
}

bool IMU3DMGX510::set_streamon(){
    //We activate data stream
        port.WriteLine(streamon);
        //3DMGX10 will answer back a message showing if any error appeared in the process
        //We must read it
        bool comprobacion = port.CheckLine(respuestacorrectastreamonoff,streamon);
        if (comprobacion == 1){
//            cout << "Envio y respuesta correctos" << endl;
        }
    return comprobacion;
}

bool IMU3DMGX510::set_streamoff(){
    //We activate data stream
    port.WriteLine(streamoff);
    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    bool comprobacion = port.CheckLine(respuestacorrectastreamonoff,streamoff);
    if (comprobacion == 1){
        //            cout << "Envio y respuesta correctos" << endl;
    }
    return comprobacion;
}

bool IMU3DMGX510::set_reset(){
    //We activate data stream
    port.WriteLine(reset);
    //3DMGX10 will answer back a message showing if any error appeared in the process
    //We must read it
    bool comprobacion = port.CheckLine(respuestacorrectareset,reset);
    if (comprobacion == 1){
        //            cout << "Envio y respuesta correctos" << endl;
    }
    return comprobacion;
}

bool IMU3DMGX510::set_devicetogetgyroacc(){
    //We will prepare our device to get gyros and accs values
    //Freq will be introduced by user (1Hz or 100Hz atm)
    bool comprobacion;
    if(freq==1){
        port.WriteLine(gyracc1);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc1);
    }else if (freq == 50){
        port.WriteLine(gyracc50);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc50);
    }else if (freq == 100){
        port.WriteLine(gyracc100);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc100);
    }else if (freq == 500){
        port.WriteLine(gyracc500);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc500);
    }else if (freq == 1000){
        port.WriteLine(gyracc1000);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc1000);
    }else{
        port.WriteLine(gyracc100);
        comprobacion = port.CheckLine(respuestacorrectaajustes,gyracc100);
    }

    if (comprobacion == 1){
        //        cout << "Envio y respuesta correctos" << endl;
    }
    return comprobacion;
}

bool IMU3DMGX510::set_devicetogetgyro(){
    //We will prepare our device to get gyros and accs values
    //Freq will be introduced by user (1Hz or 100Hz atm)
    bool comprobacion;
    if (freq==1){
        port.WriteLine(imudata1);
        comprobacion = port.CheckLine(respuestacorrectaajustes,imudata1);
    }else if (freq==100){
        port.WriteLine(imudata100);
        comprobacion = port.CheckLine(respuestacorrectaajustes,imudata100);
    }else if (freq==1000){
        port.WriteLine(imudata1000);
        comprobacion = port.CheckLine(respuestacorrectaajustes,imudata1000);
                    printf(">>>1000 \n");
    }else{
        port.WriteLine(imudata100);
        comprobacion = port.CheckLine(respuestacorrectaajustes,imudata100);
    }

    if (comprobacion == 1){
        cout << "Envio y respuesta correctos" << endl;
    }
    return comprobacion;
}


// -----------------------------------------------------------------------


// -------------------------  Getting data  ------------------------------

std::tuple <float, float, float> IMU3DMGX510::get_gyroPolling() {

    //We send data to set 3DMGX10 to polling mode
    port.WriteLine(polling);

    string reading;
    float gyroxvalue, gyroyvalue, gyrozvalue;

    //3DMGX10 will answer back a message with gyro values
    //We must read it
    reading = port.GetNumberofChars(20);

        //X
        ulf x;
        reading.substr(6,4);
        std::string str =hex(reading.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> x.ul;
        gyroxvalue = x.f;

        //Y
        ulf y;
        std::string str1 =hex(reading.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> y.ul;
        gyroyvalue = y.f;

        //Z
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
        comp=0;
        fin=0;

        do{
            c = port.GetChar();
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

        char descriptor = port.GetChar();
        reading+=descriptor;

        char longitud = port.GetChar();
        reading+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            c = port.GetChar();
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

            estimador.update(period,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
        }
    }
    }else{

        //We send data to set 3DMGX10 to polling mode
        port.WriteLine(polling);

        comp=0;
        fin=0;

        do{
            c = port.GetChar();
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

        char descriptor = port.GetChar();
        reading+=descriptor;

        char longitud = port.GetChar();
        reading+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            c = port.GetChar();
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

            estimador.update(period,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
        }
     }

    estimation[0]=estimador.eulerRoll();
    estimation[1]=estimador.eulerPitch();
    return estimation;
}

std::tuple <double*,double*,double*> IMU3DMGX510::get_gyroStreaming(int samples){

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
             c = port.GetChar();
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

         descriptor = port.GetChar();
         answer+=descriptor;

         longitud = port.GetChar();
         answer+=longitud;

         for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
             c = port.GetChar();
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
std::tuple <double*,double*,double,double> IMU3DMGX510::get_euleranglesStreaming(int samples){

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
            c = port.GetChar();
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

        descriptor = port.GetChar();
        answer+=descriptor;

        longitud = port.GetChar();
        answer+=longitud;

        for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
            c = port.GetChar();
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

                estimador.update(period,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
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

double* IMU3DMGX510::EulerAngles() {

    string answer, str, str1, str2, str3, str4, str5;
    char c, longitud, descriptorgeneral, descriptormsg1, descriptormsg2;
    static double EulerAngles[2];

    //Reset of the variables to avoid infinite loops.
    answer.clear();
    str.clear();
    str1.clear();
    str2.clear();
    str3.clear();
    str4.clear();
    str5.clear();
    int comp = 0;
    int fin = 0;
    ulf accx,accy,accz,gyrox,gyroy,gyroz;
    double f=0;
    double f1=0;
    double f2=0;
    double f3=0;
    double f4=0;
    double f5=0;
    c = '0';
    longitud = '0';
    descriptorgeneral = '0';
    descriptormsg1 = '0';
    descriptormsg2 = '0';

    //Start of the loop
    do{
        c = '0';
        c = port.GetChar();
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
                answer.clear();
            }
            break;}

        default:{
            answer.clear();
            break;}
        }
    }while(fin==0);

    descriptorgeneral = port.GetChar();
    answer+=descriptorgeneral;

    longitud = port.GetChar();
    answer+=longitud;

    for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
        c = port.GetChar();
        answer+=c;
    }

//    75 65 80 1C 0E 04 BB 16 DF 76 3C 2F B0 49 3F 80 1A 85 0E 05 BB 5E 50 00 B9 B3 AE D4 BA FB B0 C6 05 FF

//    75 65 80 1C 0E 04 F7 E4 BB 13 A5 1A BB 0A FF A6 BA 94 75 65 80 1C 0E 04 3B 1A 8C 74 3C 1E 34 89 3F 7F


    if (int(longitud) == 28 && int(descriptorgeneral) == -128 ){

        descriptormsg1 = answer.at(5);
        descriptormsg2 = answer.at(19);

        if ( int(descriptormsg1) == 4 && int(descriptormsg2) == 5){

        str =hex(answer.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> accx.ul;
        f = accx.f;

        str1 =hex(answer.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> accy.ul;
        f1 = accy.f;

        str2 =hex(answer.substr(14,4));
        std::stringstream ss2(str2);
        ss2 >> std::hex >> accz.ul;
        f2 = accz.f;

        str3 =hex(answer.substr(20,4));
        std::stringstream ss3(str3);
        ss3 >> std::hex >> gyrox.ul;
        f3 = gyrox.f;

        str4 =hex(answer.substr(24,4));
        std::stringstream ss4(str4);
        ss4 >> std::hex >> gyroy.ul;
        f4 = gyroy.f;

        str5 =hex(answer.substr(28,4));
        std::stringstream ss5(str5);
        ss5>> std::hex >> gyroz.ul;
        f5 = gyroz.f;

        estimador.update(period,f3,f4,f5,f*9.81,f1*9.81,f2*9.81,0,0,0);
        EulerAngles[0]=estimador.eulerRoll() - rolloffset; //rads
        EulerAngles[1]=estimador.eulerPitch() - pitchoffset; //rads

        EulerAngles[0] = EulerAngles[0]*180/M_PI; //rad to degrees
        EulerAngles[1] = EulerAngles[1]*180/M_PI; //rad to degrees

        }else{
            cout << "Bad" << endl;
        }

    }else {
        cout << "Bad" << endl;
    }
    answer.clear();
    return EulerAngles;
}
double* IMU3DMGX510::GyroData(){

    //Decl. of the variables
    char c;
    string answer;
    char descriptor;
    char longitud;
    static double GyrosData[3];

    //Reset of the variables to avoid infinite loops.
    answer.clear();
    int comp=0;
    int fin=0;

    do{
        c = port.GetChar();
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

    descriptor = port.GetChar();
    answer+=descriptor;

    longitud = port.GetChar();
    answer+=longitud;

    for (int j = 0 ; j<= ((int)longitud + 1) ; j++){
        c = port.GetChar();
        answer+=c;
    }

    if (int(longitud) == 14){ //It must be 14
        //X
        ulf x;
        std::string str =hex(answer.substr(6,4));
        std::stringstream ss(str);
        ss >> std::hex >> x.ul;
        float f = x.f;
        GyrosData[0] = f;

        //y
        ulf y;
        std::string str1 =hex(answer.substr(10,4));
        std::stringstream ss1(str1);
        ss1 >> std::hex >> y.ul;
        float f1 = y.f;
        GyrosData[1] = f;

        //z
        ulf z;
        std::string str2 =hex(answer.substr(14,4));
        std::stringstream ss2(str2);
        ss2 >> std::hex >> z.ul;
        float f2 = z.f;
        GyrosData[2] = f;
    }

return GyrosData;
}

// -----------------------------------------------------------------------
