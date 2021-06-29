#include "mocapdevice.hpp"

sFrameOfMocapData* MOCAPdevice::frame = NULL;	//static attribute frame is declared as Null

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
    printf("Yarp port: %s/out has been correctly opened", nameyarpoutport.c_str());

    g_connectParams.connectionType = kDefaultConnectionType;
	g_connectParams.serverAddress = serverIP.c_str();
	
	// set the frame callback handler
    g_pClient->SetFrameReceivedCallback(dataHandler, g_pClient);	// this function will receive data

    int iResult;

    // Connect to Motive
    iResult = connectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }
	
	return 0;
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
            if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
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
            }
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
        printf("Client IP:%s\n", g_connectParams.localAddress);
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
	
    struct Quaternions quaternAngles = {};
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
                        quaternAngles.qw += nowFrame->RigidBodies[iBody].qw;
                        quaternAngles.qx += nowFrame->RigidBodies[iBody].qx;
                        quaternAngles.qy += nowFrame->RigidBodies[iBody].qy;
                        quaternAngles.qz += nowFrame->RigidBodies[iBody].qz;
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
    quaternAngles.qw = quaternAngles.qw / calibrNumSamples;
    quaternAngles.qx = quaternAngles.qx / calibrNumSamples;
    quaternAngles.qy = quaternAngles.qy / calibrNumSamples;
    quaternAngles.qz = quaternAngles.qz / calibrNumSamples;

	//Print quaternions taken for calibration
    printf("\n\tqx   \tqy   \tqz   \tqw\n");
    printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
        quaternAngles.qx,
        quaternAngles.qy,
        quaternAngles.qz,
        quaternAngles.qw);
	
	//Building rotation matrix
    createRotMatrixQuaternion(quaternAngles);
	
	//Print rotation matrix
    for(int x=0;x<4;x++)
    {
        for(int y=0;y<4;y++)
        {
            printf("\t %3.2f", calibrRotMatrix[x][y]);
        }
		printf("\n");
    }
	
	printf("Calibration Done\n");
}

void MOCAPdevice::createRotMatrixQuaternion(struct Quaternions qAngles) {
    double rotMat[4][4] = {{qAngles.qw,    -qAngles.qz,    qAngles.qy,     qAngles.qx},
                            {qAngles.qz,    qAngles.qw,     -qAngles.qx,    qAngles.qy},
                            {-qAngles.qy,   qAngles.qx,     qAngles.qw,     qAngles.qz},
                            {-qAngles.qx,   -qAngles.qy,    -qAngles.qz,    qAngles.qw}};

    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            calibrRotMatrix[row][col] = rotMat[row][col];
        }
    }
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

struct EulerAngles MOCAPdevice::applyCalibration(struct Quaternions quatern) {
    struct Quaternions finalQuat;

    //  Apply rotation correction for the orientation angles
    finalQuat.qx = (calibrRotMatrix[0][0] * quatern.qx + calibrRotMatrix[0][1] * quatern.qy + calibrRotMatrix[0][2] * quatern.qz + calibrRotMatrix[0][3] * quatern.qw);
    finalQuat.qy = (calibrRotMatrix[1][0] * quatern.qx + calibrRotMatrix[1][1] * quatern.qy + calibrRotMatrix[1][2] * quatern.qz + calibrRotMatrix[1][3] * quatern.qw);
    finalQuat.qz = (calibrRotMatrix[2][0] * quatern.qx + calibrRotMatrix[2][1] * quatern.qy + calibrRotMatrix[2][2] * quatern.qz + calibrRotMatrix[2][3] * quatern.qw);
    finalQuat.qw = (calibrRotMatrix[3][0] * quatern.qx + calibrRotMatrix[3][1] * quatern.qy + calibrRotMatrix[3][2] * quatern.qz + calibrRotMatrix[3][3] * quatern.qw);

    return converToEuler(finalQuat, 0); // orientation in euler angles (sexagesimal degrees)
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




