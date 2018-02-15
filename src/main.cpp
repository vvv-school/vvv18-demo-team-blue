

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
    LISTENING,
    DETECTING_OBJECT,
    POINTING_AT_OBJECT,
    TALKING,
    HIGH_FIVE,
    DETECTING_FORCES,
    REACTING_TO_FEEDBACK,
    END
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

    State state_val;
    bool closing;
    yarp::os::Mutex mutex;

    const short FAILURE_THRESHOLD = 5;

    yarp::sig::Vector object_position;
    int object_id;
    std::string text_to_talk;
    std::string expression;
    short fail_count;
    inline void setState(State s) {
        mutex.lock();
        // set the state to given state if it not END. this avoids overrwriting of inteerupt update
        state_val = state_val == END ? END : s;
        mutex.unlock();
    }

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
    bool execute(int s)
    {
        //default value of s is INIT
        state_val = State(s);
        fail_count = 0;
        expression = "sad";
        short pointing_failure = 0;

        while(state_val != END){

            if(state_val == INIT && fail_count < FAILURE_THRESHOLD  && init()) {
                setState(LISTENING);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "Sorry, I failed to initialize";
                setState(TALKING);
            }

            if(state_val == LISTENING && fail_count < FAILURE_THRESHOLD  && listenCommand()) {
                setState(DETECTING_OBJECT);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "I tried few times, but I cannot understand you.";
                setState(TALKING);
            }

            if(state_val == DETECTING_OBJECT  && fail_count < FAILURE_THRESHOLD && detectObject(object_id)) {
                setState(POINTING_AT_OBJECT);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "I tried few times, but I cannot understand you.";
                setState(TALKING);
            }

            if(state_val == POINTING_AT_OBJECT  && pointing_failure < FAILURE_THRESHOLD && pointAtObject(object_position) ) {
                setState(TALKING);
                fail_count = 0;
                pointing_failure = 0;
            }
            else {
                ++pointing_failure;
                setState(DETECTING_FORCES);
            }

            if(state_val == TALKING && fail_count < FAILURE_THRESHOLD  && pointAtObject(object_position)) {
                setState(HIGH_FIVE);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "How can I fail to talk and talk this. you better check your state machine";
                setState(TALKING);
            }

            if(state_val == HIGH_FIVE && fail_count < FAILURE_THRESHOLD && high_five(true) ) {
                setState(DETECTING_FORCES);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "I cannot raise my hand";
                setState(TALKING);
            }

            if(state_val == DETECTING_FORCES && fail_count < FAILURE_THRESHOLD  && detect_forces()) {
                setState(REACTING_TO_FEEDBACK);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "I was waiting for 5 minutes to get a high five. Do you not like me?";
                setState(TALKING);
            }

            if(state_val ==  REACTING_TO_FEEDBACK && fail_count < FAILURE_THRESHOLD && display_expression(expression) ) {
                setState(END);
                fail_count = 0;
            }
            else {
                ++fail_count;
                text_to_talk = "We came, we saw, we conquered";
                setState(END);
            }


        }

        if(fail_count == FAILURE_THRESHOLD){
            return false;
        }

        return true;
    }

    bool init()
    {
        yInfo()<<"StateMachine::init : Executing Init";
        return true;
    }

    bool listenCommand(){

        yarp::os::Bottle command, response;
        command.addString("listen");
        voiceCommandPort.write(command,response);
        if(response.size() == 1){
            object_id = response.get(0).asInt();
            return true;
        }
        return false;
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

    bool detect_forces(){

        yarp::os::Bottle command, response;
        command.addString("detectForces");

        forceFeedbackPort.write(command,response);
        int status = NO_FEEDBACK;
        if(response.size() == 1){
            status = response.get(0).asInt();
        }

        return status;
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
        // close everything
        rpcPort.close();
        voiceCommandPort.close();
        detectorPort.close();
        pointObjectPort.close();
        forceFeedbackPort.close();
        behaviorPort.close();

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

    bool interruptModule(){
        closing = true;
        setState(END);

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
