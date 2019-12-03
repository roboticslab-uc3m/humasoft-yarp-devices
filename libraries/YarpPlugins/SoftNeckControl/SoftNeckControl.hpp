// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SOFT_NECK_CONTROL_HPP__
#define __SOFT_NECK_CONTROL_HPP__

#include <mutex>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortReaderBuffer.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>

#include <ICartesianControl.h>

#define DEFAULT_PREFIX "/SoftNeckControl"
#define DEFAULT_REMOTE_ROBOT "/teo/head"
#define DEFAULT_REMOTE_SERIAL "/softimu"
#define DEFAULT_SERIAL_TIMEOUT 0.1 // seconds
#define DEFAULT_CMC_PERIOD 0.05 // seconds
#define DEFAULT_WAIT_PERIOD 0.03 // seconds
#define NUM_ROBOT_JOINTS 3

namespace humasoft
{

/**
 * @ingroup YarpPlugins
 * @defgroup SoftNeckControl
 *
 * @brief Contains humasoft::SoftNeckControl.
 */

/**
 * @ingroup SoftNeckControl
 * @brief Responds to streaming serial bottles.
 */
class SerialStreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    SerialStreamResponder(double timeout);
    void onRead(yarp::os::Bottle & b);
    bool getLastData(std::vector<double> & data);

private:

    const double timeout;
    double localArrivalTime;
    std::vector<double> data;
    mutable std::mutex mutex;
};

/**
 * @ingroup SoftNeckControl
 * @brief The SoftNeckControl class implements ICartesianControl.
 */
class SoftNeckControl : public yarp::dev::DeviceDriver,
                        public roboticslab::ICartesianControl,
                        public yarp::os::PeriodicThread
{
public:

    SoftNeckControl() : yarp::os::PeriodicThread(DEFAULT_CMC_PERIOD),
                        iControlMode(0),
                        iEncoders(0),
                        iPositionControl(0),
                        iPositionDirect(0),
                        currentState(VOCAB_CC_NOT_CONTROLLING),
                        cmcSuccess(true),
                        streamingCommand(VOCAB_CC_NOT_SET),
                        serialStreamResponder(DEFAULT_SERIAL_TIMEOUT)
    {}

    // -- ICartesianControl declarations. Implementation in ICartesianControlImpl.cpp --

    virtual bool stat(std::vector<double> & x, int * state = 0, double * timestamp = 0);
    virtual bool inv(const std::vector<double> & xd, std::vector<double> & q);
    virtual bool movj(const std::vector<double> & xd);
    virtual bool relj(const std::vector<double> & xd);
    virtual bool movl(const std::vector<double> & xd);
    virtual bool movv(const std::vector<double> & xdotd);
    virtual bool gcmp();
    virtual bool forc(const std::vector<double> & td);
    virtual bool stopControl();
    virtual bool wait(double timeout);
    virtual bool tool(const std::vector<double> & x);
    virtual bool act(int command);
    virtual void twist(const std::vector<double> & xdot);
    virtual void pose(const std::vector<double> & x, double interval);
    virtual void movi(const std::vector<double> & x);
    virtual bool setParameter(int vocab, double value);
    virtual bool getParameter(int vocab, double * value);
    virtual bool setParameters(const std::map<int, double> & params);
    virtual bool getParameters(std::map<int, double> & params);

    // -------- PeriodicThread declarations. Implementation in PeriodicThreadImpl.cpp --------

    virtual void run();

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

private:

    int getCurrentState() const;
    void setCurrentState(int value);
    bool presetStreamingCommand(int command);
    bool setControlModes(int mode);
    void computeIsocronousSpeeds(const std::vector<double> & q, const std::vector<double> & qd, std::vector<double> & qdot);
    void handleMovj();

    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl * iPositionControl;
    yarp::dev::IPositionDirect * iPositionDirect;

    yarp::os::BufferedPort<yarp::os::Bottle> serialPort;
    SerialStreamResponder serialStreamResponder;

    int currentState;
    bool cmcSuccess;
    int streamingCommand;
    std::vector<double> qRefSpeeds;
    mutable std::mutex stateMutex;
};

} // namespace humasoft

#endif // __SOFT_NECK_CONTROL_HPP__
