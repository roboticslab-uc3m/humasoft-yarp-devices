#include "mocapdevice.hpp"

// ------------------- DeviceDriver Related ------------------------------------

bool MOCAPdevice::open(yarp::os::Searchable & config)
{
    nameyarpoutport = config.check("sendport", yarp::os::Value(DEFAULT_OUTPORT), "Local outport yarp").asString();
    serverIP = config.check("serverIP", yarp::os::Value(DEFAULT_SERVERIP),"Name of the IP adress where Mocap software is connected and streaming data to").asString().c_str();
	cmcPeriod = config.check("cmcPeriod", yarp::os::Value(DEFAULT_CMC_PERIOD), "Thread period (seconds)").asFloat64();
    sensoredRigidBodyID = config.check("rigidBodyID",yarp::os::Value(DEFAULT_RIGIDBODYID), "ID of the desired sensored rigidBody").asInt();
    calibrNumSamples = config.check("calibrate",yarp::os::Value(DEFAULT_CALIBRNUMSAMPLES), "Number of frames to consider to calculate the calibration").asInt();
    outputYarpEuler = config.check("out", yarp::os::Value(DEFAULT_OUTPUTYARPANGLES), "Euler angles and the sending order to the yarp port").asString().c_str();

    if(config.check("help"))
    {
        printf("----------\n");
        printf("--sendport : Local outport yarp\n");
        printf("--serverIP : Name of the IP adress where Mocap software is connected and streaming data to\n");
        printf("--cmcPeriod : Thread period (seconds)\n");
        printf("--rigidBodyID : ID of the desired sensored rigidBody\n");
        printf("--calibrate : Number of frames to consider to calculate the calibration\n");
        printf("--out : Euler angles and the sending order to the yarp port\n");
        printf("----------\n\n");
        return 0;
    }

    if (cmcPeriod != DEFAULT_CMC_PERIOD)
    {
        yarp::os::PeriodicThread::setPeriod(cmcPeriod);
    }

    if(config.check("calibrate"))
    {
        calibration = true;
    }
	 
    //Mocap initilization
    setupMOCAP();
    getDataDescriptions();
	 
    if (calibration == true)
	{
        printf("Starting calibration\n");
        calibrate();
	}

    return yarp::os::PeriodicThread::start();
}

// -----------------------------------------------------------------------------

bool MOCAPdevice::close()
{
	// Done - clean up.
	if (g_pClient)
	{
		g_pClient->Disconnect();
		delete g_pClient;
		g_pClient = NULL;
	}
	yarpPort.close();
    PeriodicThread::stop();
	return ErrorCode_OK;
}

// -----------------------------------------------------------------------------
