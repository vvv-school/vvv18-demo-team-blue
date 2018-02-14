

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

enum FORCE_FEEDBACK{
    NO_FEEDBACK=-1,
    NEGATIVE_FEEDBACK,
    POSITIVE_FEEDBACK
};

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
    yarp::os::RpcClient pointObjectPort;
    yarp::os::RpcClient forceFeedbackPort;
    yarp::os::RpcClient behaviorPort;


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

    int listenCommand(){

        yarp::os::Bottle command, response;
        command.addString("listen");
        voiceCommandPort.write(command,response);
        if(response.size() == 1){
            return response.get(0).asInt();
        }
        return -1;    // if we
    }

    bool detectObject(int object_id){
        yInfo()<<"StateMachine::detectObject : Detecting object";
        yarp::os::Bottle command, response;
        command.addString("detect");
        command.addInt(object_id);
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

    bool pointAtObject(yarp::sig::Vector position){

        if (position.size() != 3){
            yInfo()<<"StateMachine::pointAtObject : Position should be a vector of 3 points";
            return false;
        }
        yarp::os::Bottle command, response;
        command.addString("point");
        command.addDouble(position(0));
        command.addDouble(position(1));
        command.addDouble(position(2));

        pointObjectPort.write(command,response);

        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    bool talk(std::string text){

        yarp::os::Bottle command, response;
        command.addString("talk");
        command.addString(text);
        voiceCommandPort.write(command,response);
        return (response.size() == 1 ? response.get(0).asBool() : false);

    }

    bool high_five(bool val){

        yarp::os::Bottle command, response;
        std::string str = val ? "high" : "low";
        command.addString(str);

        pointObjectPort.write(command,response);

        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    int detect_forces(){

        yarp::os::Bottle command, response;
        command.addString("detectForces");

        forceFeedbackPort.write(command,response);

        return (response.size() == 1 ? response.get(0).asInt() : NO_FEEDBACK);
    }

    bool display_expression(std::string expression){

        yarp::os::Bottle command, response;
        command.addString(expression);
        behaviorPort.write(command,response);
        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    /********************************************************/
    bool quit()
    {
        closing = true;
        return true;
    }

    void initializePorts(){

        std::string robot = "/robot";
        if(!voiceCommandPort.open(robot+"/voice_proc/rpc:o") ||
                !detectorPort.open(robot+"/detector/rpc:o")  ||
                !pointObjectPort.open(robot+"/point_object/rpc:o") ||
                !forceFeedbackPort.open(robot+"/force_feedback/rpc:o") ||
                !behaviorPort.open(robot+"/behavior/rpc:o")){
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

        initializePorts();

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
