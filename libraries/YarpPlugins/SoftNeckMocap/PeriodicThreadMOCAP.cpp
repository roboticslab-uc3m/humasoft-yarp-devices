#include "mocapdevice.hpp"

// This method will be run in the background to get data from the Mocap frame and publish it in Yarp

void MOCAPdevice::run()
{
    if (MOCAPdevice::frame!= nullptr)
    //if (false)
	{
		// labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
		printf("Markers [Count=%d]\n\n", MOCAPdevice::frame->nLabeledMarkers);
		// Rigid Bodies
		printf("Rigid Bodies [Count=%d]\n", MOCAPdevice::frame->nRigidBodies);
		for (int i = 0; i < MOCAPdevice::frame->nRigidBodies; i++)
		{
			if (MOCAPdevice::frame->RigidBodies[i].ID == sensoredRigidBodyID)
			{
				// params
				// 0x01 : bool, rigid body was successfully tracked in this MOCAPdevice::frame
				bool bTrackingValid = MOCAPdevice::frame->RigidBodies[i].params & 0x01;

				printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", MOCAPdevice::frame->RigidBodies[i].ID, MOCAPdevice::frame->RigidBodies[i].MeanError, bTrackingValid);

				struct EulerAngles rotAngles;
				
				if (calibration  == 1)
				{
                    rotAngles = applyCalibration(getBodyQuaternions(MOCAPdevice::frame->RigidBodies[i]));
				}
                else rotAngles = getBodyRollPitchYaw(MOCAPdevice::frame->RigidBodies[i]);

				//Position and orientation data are showed on screen
				printf("\tx[m] \ty[m] \tz[m] \tRoll[deg]\tPitch[deg]\tYaw[deg]\n");
				printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f    \t%3.2f     \t%3.2f\n",
					MOCAPdevice::frame->RigidBodies[i].x,
					MOCAPdevice::frame->RigidBodies[i].y,
					MOCAPdevice::frame->RigidBodies[i].z,
					rotAngles.roll,
					rotAngles.pitch,
					rotAngles.yaw);

				//Orientation data is sended to yarp
                sendAnglesYarp(rotAngles);
				break;
			}
		}
    }
}
