#include "mocapdevice.hpp"

sFrameOfMocapData* MOCAPdevice::frame = NULL;	//static attribute frame is declared as Null
std::vector< sNatNetDiscoveredServer > MOCAPdevice::g_discoveredServers;

bool MOCAPdevice::setupMOCAP() {

    // print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    printf("NatNet ver. %d.%d.%d.%d \n", ver[0], ver[1], ver[2], ver[3]);

    // Install logging callback
    NatNet_SetLogCallback(messageHandler);

    // create NatNet client
    g_pClient = new NatNetClient();


    yarpPort.open(nameyarpoutport+"/out");
    printf("Yarp port: %s/out has been correctly opened\n", nameyarpoutport.c_str());

    if (!serverIP.empty()) {
        g_connectParams.connectionType = kDefaultConnectionType;
        g_connectParams.serverAddress = serverIP.c_str();
    }

    else
        scanForServers();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback(dataHandler, g_pClient);	// this function will receive data

    int iResult;

    // Connect to Motive
    iResult = connectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return iResult;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }

    return ErrorCode_OK;
}

void MOCAPdevice::testRequest(){
    // Send/receive test request
    void* response;
    int nBytes;
    int iResult;
    printf("Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("Received: %s", (char*)response);
    }
}

void MOCAPdevice::getDataDescriptions() {
    // Retrieve Data Descriptions from Motive
    printf("\n\nRequesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;
    int iResult;
    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        printf("Unable to retrieve Data Descriptions.");
    }
    else
    {
        printf("Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
        for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if (sensoredRigidBodyID == -1) {
                    printf("Adquiriendo rigidBody\n");
                    sensoredRigidBodyID = pRB->ID;
                }


                if (pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL)
                {
                    for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx)
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf("\tMarker #%d:\n", markerIdx);
                        printf("\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2]);

                        if (markerRequiredLabel != 0)
                        {
                            printf("\t\tRequired active label: %d\n", markerRequiredLabel);
                        }
                    }
                }
            }
            /*else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for (int i = 0; i < pMS->nMarkers; i++)
                    printf("%s\n", pMS->szMarkerNames[i]);

            }
            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
                printf("RigidBody Parent ID : %d\n", pRB->parentID);
                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                if (sensoredRigidBodyID == -1) {
                    printf("Adquiriendo rigidBody\n");
                    sensoredRigidBodyID = pRB->ID;
                }


                if (pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL)
                {
                    for (int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx)
                    {
                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];

                        printf("\tMarker #%d:\n", markerIdx);
                        printf("\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2]);

                        if (markerRequiredLabel != 0)
                        {
                            printf("\t\tRequired active label: %d\n", markerRequiredLabel);
                        }
                    }
                }
            }
            else
            {
                printf("Unknown data type.");
                // Unknown
            }*/
        }
    }
}

int MOCAPdevice::connectClient() {

    // Release previous server
    g_pClient->Disconnect();
    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect(g_connectParams);
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded
        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset(&g_serverDescription, 0, sizeof(g_serverDescription));
        ret = g_pClient->GetServerDescription(&g_serverDescription);
        if (ret != ErrorCode_OK || !g_serverDescription.HostPresent)
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("\n Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
            g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
            g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Server IP:%s\n", g_connectParams.serverAddress);
    }

    return ErrorCode_OK;
}

void MOCAPdevice::resetClient() {
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = g_pClient->Disconnect();
    if (iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect(g_connectParams);
    if (iSuccess != 0)
        printf("error re-initting Client\n");
}

void MOCAPdevice::calibrate() {

    unsigned int iSample = 0;

    int32_t iFrame = 0;

    //Samples are taken
    do
    {
        if (MOCAPdevice::frame!= nullptr)
        {

            sFrameOfMocapData* nowFrame = MOCAPdevice::frame;
            if (iFrame != nowFrame->iFrame)  //If a new frame is handled, then data sample is taken
            {
                for (int iBody = 0; iBody < nowFrame->nRigidBodies; iBody++)
                {
                    if (nowFrame->RigidBodies[iBody].ID == sensoredRigidBodyID)
                    {
                        float progress = (iSample*100)/calibrNumSamples;
                        printf("\rSampling.....%.0f%%", progress);
                        quatInit.qw += nowFrame->RigidBodies[iBody].qw;
                        quatInit.qx += nowFrame->RigidBodies[iBody].qx;
                        quatInit.qy += nowFrame->RigidBodies[iBody].qy;
                        quatInit.qz += nowFrame->RigidBodies[iBody].qz;
                        iSample++;
                        break;
                    }
                }
            }
            iFrame = nowFrame->iFrame;
        }
        else
            continue;
    }
    while(iSample < calibrNumSamples);

    //Average of the samples taken is made
    quatInit.qw = quatInit.qw / calibrNumSamples;
    quatInit.qx = quatInit.qx / calibrNumSamples;
    quatInit.qy = quatInit.qy / calibrNumSamples;
    quatInit.qz = quatInit.qz / calibrNumSamples;

    //Print quaternions taken for calibration
    printf("\n\tqx   \tqy   \tqz   \tqw\n");
    printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
        quatInit.qx,
        quatInit.qy,
        quatInit.qz,
        quatInit.qw);

    printf("Calibration Done\n");
}

float MOCAPdevice::get_frameRate(NatNetClient *g_pClient) {
    void* pResult;
    int nBytes = 0;
    ErrorCode ret = ErrorCode_OK;
    // get mocap frame rate
    ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
    if (ret == ErrorCode_OK)
    {
        float fRate = *((float*)pResult);
        return fRate;
    }
    else
    {
        printf("Error getting frame rate.\n");
        return -1;
    }
}

// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV MOCAPdevice::messageHandler(Verbosity msgType, const char* msg) {
    // Optional: Filter out debug messages
    if (msgType < Verbosity_Info)
    {
        return;
    }

    printf("\n[NatNetLib]");

    switch (msgType)
    {
    case Verbosity_Debug:
        printf(" [DEBUG]");
        break;
    case Verbosity_Info:
        printf("  [INFO]");
        break;
    case Verbosity_Warning:
        printf("  [WARN]");
        break;
    case Verbosity_Error:
        printf(" [ERROR]");
        break;
    default:
        printf(" [?????]");
        break;
    }

    printf(": %s\n", msg);
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV MOCAPdevice::dataHandler(sFrameOfMocapData* data, void* pUserData) {
    MOCAPdevice::frame = data;
}

void NATNET_CALLCONV MOCAPdevice::serverDiscoveredCallback(const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext) {
    char serverHotkey = '.';
    if (MOCAPdevice::g_discoveredServers.size() < 9)
    {
        serverHotkey = static_cast<char>('1' + MOCAPdevice::g_discoveredServers.size());
    }

    const char* warning = "";

    if (pDiscoveredServer->serverDescription.bConnectionInfoValid == false)
    {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
    }

    printf("[%c] %s %d.%d at %s%s\n",
        serverHotkey,
        pDiscoveredServer->serverDescription.szHostApp,
        pDiscoveredServer->serverDescription.HostAppVersion[0],
        pDiscoveredServer->serverDescription.HostAppVersion[1],
        pDiscoveredServer->serverAddress,
        warning);

    MOCAPdevice::g_discoveredServers.push_back(*pDiscoveredServer);
}

void MOCAPdevice::scanForServers() {
    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.

    // Do asynchronous server discovery.
    printf("Looking for servers on the local network.\n");

    NatNetDiscoveryHandle discovery;
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
    NatNet_CreateAsyncServerDiscovery(&discovery, serverDiscoveredCallback);

    const size_t serverIndex = 0; // First server detected will be selected

    //An initial wait is necessary until at least one server is found.
    while(!(serverIndex < MOCAPdevice::g_discoveredServers.size())) {
        printf("\rScanning");
    }

    if (serverIndex < MOCAPdevice::g_discoveredServers.size())
    {

        const sNatNetDiscoveredServer& discoveredServer = MOCAPdevice::g_discoveredServers[serverIndex];
        if (discoveredServer.serverDescription.bConnectionInfoValid)
        {
            // Build the connection parameters.

#ifdef _WIN32
            _snprintf_s(
#else
            snprintf(
#endif
                g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                discoveredServer.serverDescription.ConnectionMulticastAddress[3]
            );

            g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
            //g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
            //g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
            g_connectParams.serverAddress = discoveredServer.serverAddress;
            //g_connectParams.localAddress = discoveredServer.localAddress;
            //g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
        }
        else
        {
            printf("\n--------------->Inside--------\n\n");
            // We're missing some info because it's a legacy server.
            // Guess the defaults and make a best effort attempt to connect.
            g_connectParams.connectionType = kDefaultConnectionType;
            g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
            g_connectParams.serverDataPort = 0;
            g_connectParams.serverAddress = discoveredServer.serverAddress;
            g_connectParams.localAddress = discoveredServer.localAddress;
            g_connectParams.multicastAddress = NULL;
        }
    }

    NatNet_FreeAsyncServerDiscovery(discovery);
}

struct EulerAngles MOCAPdevice::applyCalibration(struct Quaternions actualQuat) {
    struct Quaternions invQuatInit;
    struct Quaternions relatQuat;

    invQuatInit.qw = quatInit.qw;
    invQuatInit.qx = -quatInit.qx;
    invQuatInit.qy = -quatInit.qy;
    invQuatInit.qz = -quatInit.qz;

    relatQuat.qw = (invQuatInit.qw  * actualQuat.qw - invQuatInit.qx  * actualQuat.qx - invQuatInit.qy  * actualQuat.qy - invQuatInit.qz  * actualQuat.qz);
    relatQuat.qx = (invQuatInit.qw  * actualQuat.qx + invQuatInit.qx  * actualQuat.qw + invQuatInit.qy  * actualQuat.qz - invQuatInit.qz  * actualQuat.qy);
    relatQuat.qy = (invQuatInit.qw  * actualQuat.qy - invQuatInit.qx  * actualQuat.qz + invQuatInit.qy  * actualQuat.qw + invQuatInit.qz  * actualQuat.qx);
    relatQuat.qz = (invQuatInit.qw  * actualQuat.qz + invQuatInit.qx  * actualQuat.qy - invQuatInit.qy  * actualQuat.qx + invQuatInit.qz  * actualQuat.qw);

    return converToEuler(relatQuat, 0); // orientation in euler angles (sexagesimal degrees)
}

struct EulerAngles MOCAPdevice::converToEuler(struct Quaternions quaternAngles, bool radianes) {
    struct EulerAngles angles;
    double ori = 0;

    if (radianes ==0)
        ori = 180 / M_PI;	// sexagesimal degrees
    else
        ori = 1;

    // roll (z-axis rotation)
    double sinr_cosp = 2 * (quaternAngles.qw * quaternAngles.qz + quaternAngles.qx * quaternAngles.qy);
    double cosr_cosp = 1 - 2 * (quaternAngles.qz * quaternAngles.qz + quaternAngles.qx * quaternAngles.qx);
    angles.roll = ori * atan2(sinr_cosp, cosr_cosp);

    // pitch (x-axis rotation)
    double sinp = 2 * (quaternAngles.qw * quaternAngles.qx - quaternAngles.qy * quaternAngles.qz);
    if (std::abs(sinp) >= 1)
        angles.pitch = ori * copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    //else
    angles.pitch = ori * asin(sinp);

    // yaw (y-axis rotation)
    double siny_cosp = 2 * (quaternAngles.qw * quaternAngles.qy + quaternAngles.qx * quaternAngles.qz);
    double cosy_cosp = 1 - 2 * (quaternAngles.qx * quaternAngles.qx + quaternAngles.qy * quaternAngles.qy);
    angles.yaw = ori * atan2(siny_cosp, cosy_cosp);

    return angles;
}

struct Quaternions MOCAPdevice::getBodyQuaternions(sRigidBodyData rigidBody) {
    struct Quaternions quatern;

    quatern.qx = rigidBody.qx;
    quatern.qy = rigidBody.qy;
    quatern.qz = rigidBody.qz;
    quatern.qw = rigidBody.qw;

    return quatern;
}

struct EulerAngles MOCAPdevice::getBodyRollPitchYaw(sRigidBodyData rigidBody) {
    return converToEuler(getBodyQuaternions(rigidBody), 0);
}

void MOCAPdevice::sendAnglesYarp(struct EulerAngles eulerAngles) {
    //Euler angles are sended in the order defined in <outputYarpEuler>
    for (int i=0;i<outputYarpEuler.size();i++) {
        switch (outputYarpEuler[i]){
            case 'r':
                bot.addFloat64(eulerAngles.roll);
                break;
            case 'p':
                bot.addFloat64(eulerAngles.pitch);
                break;
            case 'y':
                bot.addFloat64(eulerAngles.yaw);
                break;
        }
    }
    yarpPort.write(bot);
    bot.clear();
}
