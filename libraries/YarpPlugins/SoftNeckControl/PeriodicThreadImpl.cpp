// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath> // std::isnormal

#include <KinematicRepresentation.hpp>
#include <yarp/os/LogStream.h>

//In order to clasify the system, we are using fstream library
#include <fstream>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    switch (getCurrentState())
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        if(controlType=="ioCoupled"){
            yInfo() <<" - Inclination Orientation Coupled Contro";
            !sensorPort.isClosed() ? handleMovjClosedLoopIOCoupled() : handleMovjOpenLoop();
        }
        else if(controlType=="ioUncoupled"){
            yInfo() <<" - Inclination Orientation Uncoupled Control";
            !sensorPort.isClosed() ? handleMovjClosedLoopIOUncoupled() : handleMovjOpenLoop();
        }
        else if(controlType=="rpUncoupled"){
            yInfo() <<" - Roll Pitch Uncoupled Control";
            !sensorPort.isClosed() ? handleMovjClosedLoopRPUncoupled() : handleMovjOpenLoop();
        }
        else if(controlType=="rpFCVel"){
            yInfo() <<" - Roll Pitch Fractional Control in Velocity Mode";
            !sensorPort.isClosed() ? handleMovjClosedLoopRPFCVel() : handleMovjOpenLoop();
        }
        else
            yInfo() <<" - Control mode not defined. Running in open loop...";
        break;
    default:
        break;
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjOpenLoop()
{
    bool done;

    if (!iPositionControl->checkMotionDone(&done))
    {
        yError() <<"Unable to query current robot state.";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
            yWarning() <<"setRefSpeeds (to restore) failed.";
        }
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjClosedLoopIOCoupled()
{
    switch (sensorType) {
        case '0':
            if (!serialStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated SparkfunIMU serial stream data.";
            } break;
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated 3DMGX510IMU stream data.";
            } break;
        case '2':
            if (!mocapStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated Mocap stream data.";
                iVelocityControl->stop();
            } break;
    }

    polarError   = targetPose[0] - x_imu[0];
    azimuthError = targetPose[1] - x_imu[1];

    if(std::abs(azimuthError)> 180 )
    {
        if(azimuthError > 0)
            azimuthError = azimuthError - 360.0;
        else
            azimuthError = azimuthError + 360.0;
    }

    // controlamos siempre en inclinación
    polarCs   = polarError   > *controllerPolar;
    if (!std::isnormal(polarCs))
    {
        polarCs = 0.0;
    }

    xd[0] = polarCs;

    /* control en orientacion solo si:
     * (inclinacion > 5)
     */
    if(targetPose[0]>5)
    {
        yInfo() <<"> Controlando en orientación";
        azimuthCs = azimuthError > *controllerAzimuth;

        if (!std::isnormal(azimuthCs))
        {
            azimuthCs = 0.0;
        }

        xd[1] = azimuthCs;
    }

    yDebug("- Polar:   target %f, sensor %f, error %f, cs: %f\n", targetPose[0], x_imu[0], polarError, polarCs);
    yDebug("- Azimuth: target %f, sensor %f, error %f, cs: %f\n", targetPose[1], x_imu[1], azimuthError, azimuthCs);

    if (!encodePose(xd, xd, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
    {
        yError() <<"encodePose failed.";
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!sendTargets(xd))
    {
        yWarning() <<"Command error, not updating control this iteration.";
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjClosedLoopIOUncoupled()
{
    double cs1; // motor izq
    double cs2; // motor der
    std::vector<int> m; // motor izq, der, tercero
    std::vector<double> cs;
    int area_c, area_d = 0; // currect area, destination area (area_p = area actual)
    cs.resize(3);

    switch (sensorType) {
        case '0':
            if (!serialStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated serial stream data.";
                iVelocityControl->stop();
            } break;
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated IMU 3dmgx510 stream data.";
                iVelocityControl->stop();
            } break;
        case '2':
            if (!mocapStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated Mocap stream data.";
                iVelocityControl->stop();
            } break;
    }

    std::vector<double> xd = targetPose;
    polarError   = (xd[0] - x_imu[0]);
    azimuthError = (xd[1] - x_imu[1]);

    if (x_imu[1]<120)  area_c=1;
    else if (x_imu[1]>240) area_c=3;
    else area_c=2;

    // Paso por cero
    if (xd[1]<120) // Area 1
    {
        area_d=1;
        m={0,1,2}; //m={1,2,0}; config anterior
        if (area_c==3) azimuthError+=360;
    }
    else if (xd[1]>240) // Area 3
    {        
        area_d=3;
        m={2,0,1}; //m={0,1,2};
        if (area_c==1) azimuthError-=360;
    }
    else // Area 2
    {
        area_d=2;
        m={1,2,0}; //m={2,0,1};
    }

    polarCs   = polarError*M_1_PI/180   > *incon;
    azimuthCs = azimuthError*M_1_PI/180 > *orcon;

    if (!std::isnormal(polarCs)) polarCs = 0;
    if (!std::isnormal(azimuthCs) || x_imu[1] <5) azimuthCs = 0;

    //ajustar solo orientación, hasta llegar al área
    if (area_d!=area_c && x_imu[0]>5) polarCs=0;

    cs[0]=(polarCs-azimuthCs); // motor izquierdo del area
    cs[1]=(polarCs+azimuthCs); //motor derecho del area
    cs[2]= - (cs[0]+cs[1])/2; //2/3

    //si no estoy en el área
    if ((area_d!=area_c) && (x_imu[0]>5)){
        if (azimuthError>0) // aumentar de area bloqueo iz
        {
            cs[0]=0;
            cs[2]=-cs[1];
        }

        else
        {
            cs[1]=0;
            cs[2]=-cs[0];
        }
    }

    printf("> sensor(i%f o%f) motors (%f %f %f)\n",x_imu[0], x_imu[1], cs[0], cs[1], cs[2]);
    if (!iVelocityControl->velocityMove(3,m.data(),cs.data()));
    {
        //yError() <<"velocityMove failed.\n");
    }
}

void SoftNeckControl::handleMovjClosedLoopRPUncoupled(){

    switch (sensorType) {
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated IMU 3dmgx510 stream data.";
                iPositionControl->stop();
            } break;
        case '2':
            if (!mocapStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated Mocap stream data.";
                iPositionControl->stop();
            } break;
    }

    // transform [roll.pitch] to [inc, ori]
    std::vector<double> io_imu(2); // inc, ori
    //io_imu[0] = sqrt(pow(x_imu[1], 2) + pow(x_imu[0], 2));
    //io_imu[1] = fmod( (360 - (atan2(-x_imu[0], x_imu[1])) * 180/M_PI), 360);

    rollError = targetPose[0] - x_imu[0];
    pitchError = targetPose[1] - x_imu[1];

    //Control process
    rollCs = fcRollPosition->OutputUpdate(rollError);
    if (!std::isnormal(rollCs))
    {
        rollCs = 0.0;
    }

    pitchCs = fcPitchPosition->OutputUpdate(pitchError);
    if (!std::isnormal(pitchCs))
    {
        pitchCs = 0.0;
    }

    mp[0] = - 0.001*(pitchCs / 1.5);
    mp[1] = - 0.001*((rollCs / 1.732) - (pitchCs / 3));
    mp[2] = - 0.001*((pitchCs / -3) - (rollCs / 1.732));

    for(int i=0; i<mp.size(); i++)
    {
       if(mp[i]<0)
       {
           mp[i] = mp[i]*0.5;
       }
    }

    tprev = tnow;
    tnow = std::chrono::system_clock::now();
    chrono::nanoseconds elapsedNanoseconds = tprev.time_since_epoch()-tnow.time_since_epoch();

    double totaltime = elapsedNanoseconds.count();

    cout << "-----------------------------\n" << endl;
    cout << "Total time: ms " << (totaltime/1000000) << endl;
    //cout << "Inclination: " << io_imu[0] << "  Orientation: " << io_imu[1] << endl;
    cout << "Roll: " << x_imu[0] << "  Pitch: " << x_imu[1] << endl;
    cout << "-> RollTarget: " << targetPose[0] << " PitchTarget:  " << targetPose[1] << " >>>>> RollError: " << rollError << "  PitchError: " << pitchError << endl;
    cout << "-> Motor positions >>>>> P1: " << mp[0] << " P2: " << mp[1] << " P3: " << mp[2] << endl;
    cout << "-----------------------------\n" << endl;

    if (!iPositionControl->positionMove(mp.data()))
    {
        yError() <<"positionMove failed.";
    }

    //Uncomment it to receive data from testing
    //testingFile << yarp::os::Time::now()*numtime << "," << targetPose[0] << "," << targetPose[1] << "," << x_imu[0] << "," << x_imu[1]<< endl;
    numtime = numtime+1;

}

// new FRACTIONAL CONTROL based in ROLL PITCH inputs using VELOCITY MODE
void SoftNeckControl::handleMovjClosedLoopRPFCVel(){

    auto start = chrono::steady_clock::now();

    switch (sensorType) {
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated IMU 3dmgx510 stream data.";
                iPositionControl->stop();
            } break;
        case '2':
            if (!mocapStreamResponder->getLastData(x_imu))
            {
                yWarning() <<"Outdated Mocap stream data.";
                iPositionControl->stop();
            } break;
    }

    // cambio signo para igualar sentido de giro de los motores y del sensor
    roll  = - x_imu[0] * M_PI/180;
    pitch = - x_imu[1] * M_PI/180;

    // transormacion de grados a radianes
    targetPose[0] = targetPose[0] * M_PI/180;
    targetPose[1] = targetPose[1] * M_PI/180;

    rollError = targetPose[0] - roll;
    pitchError = targetPose[1] - pitch;

    //Control process
    rollCs = fcRollVelocity->OutputUpdate(rollError);
    if (!std::isnormal(rollCs))
    {
        rollCs = 0.0;
    }

    pitchCs = fcPitchVelocity->OutputUpdate(pitchError);
    if (!std::isnormal(pitchCs))
    {
        pitchCs = 0.0;
    }

    // pitch, roll to velocity in meters/sec
    double T  = DEFAULT_PLATFORM_RADIUS / DEFAULT_WINCH_RADIUS;
    mv[0] =  pitchCs * T;
    mv[1] =  rollCs * T * sin(2*M_PI/3) + pitchCs * T * cos(2*M_PI/3);
    mv[2] =  rollCs * T * sin(4*M_PI/3) + pitchCs * T * cos(4*M_PI/3);


    // ----- Controller of velocity in M0

    if (!iEncoders->getEncoderSpeed(0, &cmV0))
        yError() <<"getRefVelocity failed of motor 0.";

    velError0 = mv[0] - cmV0;
    cSV0 = cntrl0->OutputUpdate(velError0);

    if (!std::isnormal(cSV0))
    {
        cSV0 = 0.0;
    }

    if (!iVelocityControl->velocityMove(0, cSV0))
        yError() <<"velocityMove failed of motor 0.";

    // ------ Controller of velocity in M1

    if (!iEncoders->getEncoderSpeed(1, &cmV1))
        yError() <<"getRefVelocity failed of motor 1.";

    velError1 = mv[1] - cmV1;
    cSV1 = cntrl1->OutputUpdate(velError1);

    if (!std::isnormal(cSV1))
    {
        cSV1 = 0.0;
    }

    if (!iVelocityControl->velocityMove(1, cSV1))
        yError() <<"velocityMove failed of motor 1.";

    // ------- Controller of velocity in M2

    if (!iEncoders->getEncoderSpeed(2, &cmV2))
        yError() <<"getRefVelocity failed of motor 2.";

    velError2 = mv[2] - cmV2;
    cSV2 = cntrl1->OutputUpdate(velError2);

    if (!std::isnormal(cSV2))
    {
        cSV2 = 0.0;
    }

    if (!iVelocityControl->velocityMove(2, cSV2))
        yError() <<"velocityMove failed of motor 2.";


    printf("Pitch target/sensor: %.4f / %.4f\n",  targetPose[0], pitch); // pitch target, pitch sensor
    printf("Roll target/sensor: %.4f / %.4f\n",  targetPose[1], roll); // roll target,  roll sensor
    printf("Vel error : %.4f, %.4f, %.4f\n", velError0, velError1, velError2); // roll target,  roll sensor
    printf("Vel motors: %.4f, %.4f, %.4f\n", cmV0, cmV1, cmV2); // roll target,  roll sensor

    auto end = chrono::steady_clock::now();
    cout << "Period: " << getPeriod() <<" sec" <<endl;
    cout << "Estimated Period since last reset: " << getEstimatedPeriod() <<" sec"<<endl;
    cout << "Elapsed time procesing iteration code : "
         << chrono::duration_cast<chrono::milliseconds>(end - start).count()
         << " ms " << endl;
    printf("------------------------------------\n");


} // end loop
