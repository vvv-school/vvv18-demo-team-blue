

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/sig/Image.h>

#include "state_machine.h"

/********************************************************/
class StateMachine : public yarp::os::RFModule,
                public state_machine
{
 /*
  * Define variables here
  */

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {

    }

    /********************************************************/
    bool execute()
    {
        return true;
    }

    /********************************************************/
    bool quit()
    {
        return true;
    }

    public:
    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
    }

    /********************************************************/
    bool close()
    {
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /********************************************************/
    bool updateModule()
    {
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP server not available!";
        return 1;
    }

}
