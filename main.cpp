

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/sig/Image.h>

#include "state_machine.h"

enum State{
    INIT=0,
    GETTING_COMMAND,
    DETECTING_OBJECT,
    POINTING_AT_OBJECT,
    TALKING,
    HIGH_FIVE,
    DETECTING_FORCES,
    REACTING_TO_FEEDBACK
};

/********************************************************/
class StateMachine : public yarp::os::RFModule,
                public state_machine
{
 /*
  * Define variables here
  */
    yarp::os::ResourceFinder *rf;
    yarp::os::RpcServer rpcPort;

    State current_state;
    bool closing;
    yarp::os::Mutex mutex;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /********************************************************/
    bool execute()
    {
        return true;
    }

    /********************************************************/
    bool quit()
    {
        closing = true;
        return true;
    }

    public:
    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;

        std::string moduleName = rf.check("name", yarp::os::Value("vino-blue"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());


        closing = false;

        attach(rpcPort);
    }

    /********************************************************/
    bool close()
    {
        mutex.lock();
        rpcPort.close();
        mutex.unlock();
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
