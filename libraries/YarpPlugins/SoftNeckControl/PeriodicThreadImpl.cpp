// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SoftNeckControl.hpp"

#include <cmath> // std::isnormal

#include <KinematicRepresentation.hpp>
#include <ColorDebug.h>

using namespace humasoft;
using namespace roboticslab::KinRepresentation;

// ------------------- PeriodicThread Related ------------------------------------

void SoftNeckControl::run()
{
    switch (getCurrentState())
    {
    case VOCAB_CC_MOVJ_CONTROLLING:
        if(controlType=="docked"){
            CD_INFO_NO_HEADER("Arrancando control acoplado\n");
            !serialPort.isClosed() ? handleMovjClosedLoopDocked() : handleMovjOpenLoop();
        }
        else if(controlType=="undocked"){
            CD_INFO_NO_HEADER("Arrancando control desacoplado\n");
            !serialPort.isClosed() ? handleMovjClosedLoopUndocked() : handleMovjOpenLoop();
        }
        else if(controlType=="newUndocked"){
            CD_INFO_NO_HEADER("Arrancando control nuevo desacoplado\n");
            !serialPort.isClosed() ? handleMovjClosedLoopNewUndocked() : handleMovjOpenLoop();
        }
        else CD_ERROR("Control mode not defined\n");                
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
        CD_ERROR("Unable to query current robot state.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (done)
    {
        setCurrentState(VOCAB_CC_NOT_CONTROLLING);

        if (!iPositionControl->setRefSpeeds(qRefSpeeds.data()))
        {
            CD_WARNING("setRefSpeeds (to restore) failed.\n");
        }
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjClosedLoopDocked()
{
    std::vector<double> x_imu;
    double polarError,
           azimuthError,
           polarCs,
           azimuthCs
           = 0.0;

    switch (sensorType) {
        case '0':
            if (!serialStreamResponder->getLastData(x_imu))
            {
                CD_WARNING("Outdated serial stream data.\n");
            } break;
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                CD_WARNING("Outdated IMU 3dmgx510 stream data.\n");
            } break;
    }

    std::vector<double> xd = targetPose;   
    polarError   = xd[0] - x_imu[0];
    azimuthError = xd[1] - x_imu[1];

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
        CD_INFO_NO_HEADER("> Controlando en orientación\n");
        azimuthCs = azimuthError > *controllerAzimuth;

        if (!std::isnormal(azimuthCs))
        {
            azimuthCs = 0.0;
        }

        xd[1] = azimuthCs;
    }

    CD_DEBUG_NO_HEADER("- Polar:   target %f, sensor %f, error %f, cs: %f\n", targetPose[0], x_imu[0], polarError, polarCs);
    CD_DEBUG_NO_HEADER("- Azimuth: target %f, sensor %f, error %f, cs: %f\n", targetPose[1], x_imu[1], azimuthError, azimuthCs);

    if (!encodePose(xd, xd, coordinate_system::NONE, orientation_system::POLAR_AZIMUTH, angular_units::DEGREES))
    {
        CD_ERROR("encodePose failed.\n");
        cmcSuccess = false;
        stopControl();
        return;
    }

    if (!sendTargets(xd))
    {
        CD_WARNING("Command error, not updating control this iteration.\n");
    }
}

// -----------------------------------------------------------------------------

void SoftNeckControl::handleMovjClosedLoopUndocked()
{
    std::vector<double> x_imu;
    double polarError,
           azimuthError,
           polarCs,
           azimuthCs
           = 0.0;

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
                CD_WARNING("Outdated serial stream data.\n");
                iVelocityControl->stop();
            } break;
        case '1':
            if (!immu3dmgx510StreamResponder->getLastData(x_imu))
            {
                CD_WARNING("Outdated IMU 3dmgx510 stream data.\n");
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
        //CD_ERROR("velocityMove failed.\n");
    }
}

void SoftNeckControl::handleMovjClosedLoopNewUndocked(){
  std::vector<double> x_imu;
  if (!immu3dmgx510StreamResponder->getLastData(x_imu))
  {
      CD_WARNING("Outdated IMU 3dmgx510 stream data.\n");
      iVelocityControl->stop();
  }


  //Testing
  double targetroll = 0.0;
  double targetpitch = 0.0;
  double errorroll = targetroll - x_imu[0];
  double errorpitch = targetpitch - x_imu[1];



  double cs1; // motor izq
  double cs2; // motor der
  std::vector<int> m; // motor izq, der, tercero
  std::vector<double> cs;
  int area_c, area_d = 0; // currect area, destination area (area_p = area actual)
  cs.resize(3);

  std::vector<double> xd = targetPose;
  rollError   = xd[0] - x_imu[0];
  pitchError = xd[1] - x_imu[1];








  printf(">>>> sensor(roll%f pitch%f)\n",x_imu[0], x_imu[1]);
  printf(">>>> error(errorroll%f errorpitch%f)\n",errorroll, errorpitch);

}
