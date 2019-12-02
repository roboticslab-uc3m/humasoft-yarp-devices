// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SOFT_NECK_CONTROL_HPP__
#define __SOFT_NECK_CONTROL_HPP__

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/PolyDriver.h>

#include <ICartesianControl.h>

#define DEFAULT_PREFIX "/softNeckControl"
#define DEFAULT_REMOTE_ROBOT "/teo/head"
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
 * @brief The SoftNeckControl class implements ICartesianControl.
 */
class SoftNeckControl : public yarp::dev::DeviceDriver,
                        public roboticslab::ICartesianControl
{
public:

    SoftNeckControl()
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

    // -------- DeviceDriver declarations. Implementation in IDeviceImpl.cpp --------

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

private:

    yarp::dev::PolyDriver robotDevice;

    yarp::dev::IControlMode * iControlMode;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl * iPositionControl;
};

} // namespace humasoft

#endif // __SOFT_NECK_CONTROL_HPP__
