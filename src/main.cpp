

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Mutex.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>
#include <yarp/sig/Vector.h>

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

    yarp::sig::Vector object_position;


public:
    yarp::os::RpcClient voiceCommandPort;
    yarp::os::RpcClient detectorPort;



    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /********************************************************/
    bool execute()
    {
        yInfo()<<"StateMachine::execute : Executing Init";
        yarp::os::Time::delay(1);
        yInfo()<<"StateMachine::execute : Listening to command";
        yarp::os::Time::delay(1);
        yInfo()<<"StateMachine::execute : Detecting object";
        return true;
    }

    bool init()
    {
        yInfo()<<"StateMachine::init : Executing Init";
        return true;
    }

    bool detectObject(int object_id){
        yInfo()<<"StateMachine::detectObject : Detecting object";
        yarp::os::Bottle command, response;
        detectorPort.write(command,response);

        ///check size before fetching
         if(response.size() !=3) {
             yInfo()<<"StateMachine::detectObject : response is of size :"<<response.size()<<" It should have 3 values";
            return false;
         }

        object_position.resize(3,0.0);
        object_position(0) = response.get(0).asDouble();
        object_position(1) = response.get(1).asDouble();
        object_position(2) = response.get(2).asDouble();

        return true;    // response from NLP should always be boolean
    }

    int listenCommand(){

        yarp::os::Bottle command, response;
        voiceCommandPort.write(command,response);
        if(response.size() == 1){
            return response.get(0).asInt();
        }
        return -1;    // if we
    }

    /********************************************************/
    bool quit()
    {
        closing = true;
        return true;
    }

    void initializePorts(){
        std::string robot = "/robot";

        int count = 0;
        if(!voiceCommandPort.open(robot+"/voice_proc/rpc:o") ||
                !detectorPort.open(robot+"/detector/rpc:o")){
            yError()<<"StateMachine::initializePorts : Initializing ports failed";
        }


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
        return !closing;
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

    StateMachine module;
    yarp::os::ResourceFinder rf;

    rf.configure(argc,argv);

    return module.runModule(rf);

}
