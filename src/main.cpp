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

//#define DEBUG true

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

    const int FAILURE_THRESHOLD = 5;

    yarp::sig::Vector object_position;
    int object_id;
    std::string text_to_talk;
    std::string expression;
    short fail_count;
    inline void setState(State s) {
        yInfo()<<"Changing state from "<<state_val<<" to "<<(int)s<<" Failure count"<<fail_count;
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
            yInfo()<< "Current State :"<< state_val;
            if(state_val == INIT && fail_count < FAILURE_THRESHOLD  && init()) {
                setState(LISTENING);
                fail_count = 0;
            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD){
                    text_to_talk = "Sorry, I failed to initialize. please try again";
                    setState(TALKING);
                }
            }

            if(state_val == LISTENING && fail_count < FAILURE_THRESHOLD  && listenCommand()) {
                setState(DETECTING_OBJECT);
                fail_count = 0;
            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD){
                    text_to_talk = "I tried few times, but I cannot understand you.";
                    setState(TALKING);
                }
            }

            if(state_val == DETECTING_OBJECT  && fail_count < FAILURE_THRESHOLD && detectObject(object_id)) {
                setState(POINTING_AT_OBJECT);
                fail_count = 0;
            }
            else {
                yInfo()<<"Failed detecting. object_id = "<<object_id<<" current state is "<<state_val<<" Failure count "<<fail_count;
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD){
                    text_to_talk = "Cannot detect the objects";
                    setState(TALKING);
                }
            }

            if(state_val == POINTING_AT_OBJECT  && pointing_failure < FAILURE_THRESHOLD && pointAtObject(object_position) ) {
                text_to_talk = "Give me a high five if that is right.";
                yarp::os::Time::delay(2.0);
                display_expression("home");
                setState(TALKING);
                fail_count = 0;
                pointing_failure = 0;
            }
            else {
                ++pointing_failure;
                if(pointing_failure >= FAILURE_THRESHOLD)
                    setState(DETECTING_FORCES);
            }

            if(state_val == TALKING && fail_count < FAILURE_THRESHOLD  && talk(text_to_talk)) {
                setState(HIGH_FIVE);
                fail_count = 0;
            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD) {
                    text_to_talk = "How can I fail to talk and talk this. you better check your state machine";
                    setState(TALKING);
                }
            }

            if(state_val == HIGH_FIVE && fail_count < FAILURE_THRESHOLD && high_five() ) {
                setState(DETECTING_FORCES);
                fail_count = 0;
            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD){
                    text_to_talk = "I cannot raise my hand";
                    setState(TALKING);
                }
            }

            if(state_val == DETECTING_FORCES && fail_count < FAILURE_THRESHOLD  && detect_forces()) {
                setState(REACTING_TO_FEEDBACK);
                fail_count = 0;
            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD){
                    text_to_talk = "I was waiting for 5 minutes to get a high five. Do you not like me?";
                    setState(TALKING);
                    expression = "sad";
                }
            }

            if(state_val ==  REACTING_TO_FEEDBACK && fail_count < FAILURE_THRESHOLD && display_expression(expression) ) {

                if(expression == "happy"){
                    text_to_talk = "We came, we saw, we conquered";
                    setState(END);
                }
                else {
                    text_to_talk = "I tried. Lets do it one more time. Show me the objects";
                    setState(DETECTING_OBJECT);
                }

                talk(text_to_talk); // send the talk command manually --- lets do it the italian way
                fail_count = 0;
                continue;       // this should not be required

            }
            else {
                ++fail_count;
                if(fail_count >= FAILURE_THRESHOLD)
                    setState(END);
            }
            yInfo()<< "inside the loop"<<" state :"<<state_val<<" failure count "<<fail_count;

        }
        yInfo()<< "outside of loop"<<" state :"<<state_val<<" failure count "<<fail_count;
        display_expression("home");

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

        yInfo()<<"StateMachine::listenCommand : Executing listenCommand";

#ifdef DEBUG
        object_id = 1;
        return true;
#endif

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

#ifdef DEBUG
        object_position.resize(3, 0.8);
        return true;
#endif
        yarp::os::Bottle command, response;
        command.addString("get3DPoint");
        command.addInt(object_id);
        detectorPort.write(command,response);
        yarp::os::Bottle *list = response.get(0).asList();
        ///check size before fetching
        if(list->size() !=3) {
            yInfo()<<"StateMachine::detectObject : response is of size :"<<list->size()<<" It should have 3 values";
            return false;
        }

        object_position.resize(3,0.0);
        object_position(0) = list->get(0).asDouble();
        object_position(1) = list->get(1).asDouble();
        object_position(2) = list->get(2).asDouble();

        return true;    // response from NLP should always be boolean
    }

    bool pointAtObject(yarp::sig::Vector position){
        if (position.size() == 3)
            return pointAtObject(position(0), position(1), position(2));
        return false;
    }

    bool pointAtObject(double x, double y, double z){

        yInfo()<<"StateMachine::pointAtObject : Pointing at object";
#ifdef DEBUG
        return true;
#endif
        yarp::os::Bottle command, response;
        command.addString("point");
        command.addDouble(x);
        command.addDouble(y);
        command.addDouble(z);

        pointObjectPort.write(command,response);

        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    bool talk(const std::string &text){

        yInfo()<<"StateMachine::talking "+text;

#ifdef DEBUG
        return true;
#endif
        yarp::os::Bottle command, response;
        command.addString("talk");
        command.addString(text);
        voiceCommandPort.write(command,response);
        return (response.size() == 1 ? response.get(0).asBool() : false);

    }

    bool high_five(){

        yInfo()<<"StateMachine::high-five";

#ifdef DEBUG
        return true;
#endif
        yarp::os::Bottle command, response;
        std::string str = "high";
        command.addString(str);

        pointObjectPort.write(command,response);

        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    bool detect_forces(){

        yInfo()<<"StateMachine::detect-forces";

#ifdef DEBUG
        expression = "happy";
        return true;
#endif

        yarp::os::Bottle command, response;
        command.addString("detectForces");

        forceFeedbackPort.write(command,response);
        int status = NO_FEEDBACK;
        if(response.size() == 1){
            status = response.get(0).asInt();
            switch (status) {
            case NO_FEEDBACK:
                expression = "sad";
                return false;
            case NEGATIVE_FEEDBACK:
                expression = "sad";
                return true;
            case POSITIVE_FEEDBACK:
                expression = "happy";
                return true;
            default:
                break;
            }
        }

        return true;
    }

    bool display_expression(const std::string &exp){

        yInfo()<<"StateMachine::display-expression";
#ifdef DEBUG
        expression = "happy";
        return true;
#endif
        yarp::os::Bottle command, response;
        command.addString(exp);
        behaviorPort.write(command,response);
        return (response.size() == 1 ? response.get(0).asBool() : false);
    }

    /********************************************************/
    bool quit()
    {

        yInfo()<<"StateMachine::quit";
        closing = true;
        return true;
    }

    void initializePorts(){

        yInfo()<<"StateMachine::initializeports";

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
