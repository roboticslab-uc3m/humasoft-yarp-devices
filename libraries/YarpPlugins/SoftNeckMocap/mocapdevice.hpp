#ifndef MOCAPDEVICE_H
#define MOCAPDEVICE_H

//Includes needed

#define _USE_MATH_DEFINES
#include <cmath>

#include <stdio.h>
#include <cstdio>

#include <unistd.h>

//YARP
#include <yarp/sig/all.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortReaderBuffer.h>
#include <yarp/os/Network.h>
#include <ColorDebug.h>
#include <ace/DEV_Connector.h>
#include <ace/TTY_IO.h>

#include <yarp/conf/version.h>
#if YARP_VERSION_MINOR >= 3
# include <yarp/dev/ISerialDevice.h>
#else
# include <yarp/dev/SerialInterfaces.h>
#endif

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>

using namespace std;

using yarp::os::Bottle;
using yarp::os::Network;

// -------------------------------------------------------

struct EulerAngles
{
    double roll, pitch, yaw;
};

struct Quaternions
{
    double qw, qx, qy, qz;
};

// ---------------   Defines of our device   ---------------
#define DEFAULT_OUTPUTYARPANGLES "rp"  // Roll - Pitch
#define DEFAULT_RIGIDBODYID -1 // Set to -1 in order to take the first detected rigid body streamed by
#define DEFAULT_CALIBRATION 0
#define DEFAULT_CALIBRNUMSAMPLES 200

#define DEFAULT_SERVERIP "" // Set to "" (empty) in order to take the first detected server
#define DEFAULT_OUTPORT "/softmocap"

//Default period time
#define DEFAULT_CMC_PERIOD 0.01 // seconds

// -------------------------------------------------------

class MOCAPdevice :  public yarp::dev::DeviceDriver,
                     public yarp::os::PeriodicThread
 {
public:
    MOCAPdevice() :  
					cmcPeriod(DEFAULT_CMC_PERIOD),
                    kDefaultConnectionType(ConnectionType_Multicast),
					serverIP(DEFAULT_SERVERIP),
					sensoredRigidBodyID(DEFAULT_RIGIDBODYID),
                    calibration(DEFAULT_CALIBRATION),
					calibrRotMatrix{},
                    calibrNumSamples(DEFAULT_CALIBRNUMSAMPLES),
                    outputYarpEuler(DEFAULT_OUTPUTYARPANGLES),
                    yarp::os::PeriodicThread(DEFAULT_CMC_PERIOD)
    {}

// -------- DeviceDriver declarations. Implementation in DeviceDriverMOCAPImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

// -------- PeriodicThread declarations. Implementation in PeriodicThreadMOCAP.cpp --------

   virtual void run();

private:

    bool setupMOCAP();
    void calibrate();

	//NatNet methods
    void getDataDescriptions();
	void testRequest();
	int connectClient();
	void resetClient();
	float get_frameRate(NatNetClient *g_pClient);
    void scanForServers();
    //(Next three methods are necessarily static with fixed parameters in order to match with the functions "NatNet_SetLogCallback", "SetFrameReceivedCallback" and "NatNet_CreateAsyncServerDiscovery")
    static void NATNET_CALLCONV messageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages 
    static void NATNET_CALLCONV dataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
    static void NATNET_CALLCONV serverDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext);

    struct EulerAngles applyCalibration(struct Quaternions quaternAngles);
	
    struct Quaternions getBodyQuaternions(sRigidBodyData rigidBody);
    struct EulerAngles getBodyRollPitchYaw(sRigidBodyData rigidBody);
    struct EulerAngles converToEuler(struct Quaternions quaternAngles, bool radianes=0);
    void createRotMatrixQuaternion(struct Quaternions qAngles);

    void sendAnglesYarp(struct EulerAngles eulerAngles);

    //NatNet variables
    NatNetClient *g_pClient;
	sNatNetClientConnectParams g_connectParams;
    const ConnectionType kDefaultConnectionType;
    sServerDescription g_serverDescription;

    static sFrameOfMocapData* frame; // Necessarily static in order to be accessed in the static method "dataHandler"
    static std::vector< sNatNetDiscoveredServer > g_discoveredServers; // Necessarily static in order to be accessed in the static method "serverDiscoveredCallback"

    std::string serverIP;
    int32_t sensoredRigidBodyID;

	//Calibration variables
    bool calibration;
	unsigned int calibrNumSamples;
    double calibrRotMatrix[4][4];
	
    //Outport to publish mocap data. This kind of port allow us to start a server in the background
    yarp::os::Network Yarp;
    yarp::os::Port yarpPort;
    std::string nameyarpoutport;
    Bottle bot;
    std::string outputYarpEuler;
	
	//PeriodicThread parameters
	double cmcPeriod;

};

// -------------------------------------------------------

#endif // MOCAPDEVICE_H
