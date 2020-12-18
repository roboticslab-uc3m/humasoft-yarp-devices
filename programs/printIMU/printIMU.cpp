#include <iostream>

#include <mutex>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/PortReaderBuffer.h>



class SerialStreamResponder : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:

    SerialStreamResponder(double timeout);
    ~SerialStreamResponder();
    void onRead(yarp::os::Bottle & b);
    bool getLastData(std::vector<double> & x);

private:

    bool accumulateStuff(const std::string & s);

    const double timeout;
    double localArrivalTime;
    std::vector<double> x;
    std::string accumulator;
    mutable std::mutex mutex;
    //SystemBlock * polarFilterSensor;
    //SystemBlock * azimuthFilterSensor;
};

void SerialStreamResponder::onRead(yarp::os::Bottle & b)
{
    std::lock_guard<std::mutex> lock(mutex);

    if (b.size() == 0)
    {
        return;
    }

    if (accumulateStuff(b.get(0).asString()))
    {
        localArrivalTime = yarp::os::Time::now();
    }
}


int main()
{
	return 0;
}
