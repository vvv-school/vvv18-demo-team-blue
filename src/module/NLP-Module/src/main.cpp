#include <iostream>
#include <string>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/all.h>

using namespace std;
using namespace yarp::os;

class NLPModule:public RFModule
{
    RpcServer handlerPort; //a port to handle messages

    RpcClient speechRecog_port;
    RpcClient iSpeakHelper_port;

    std::vector<std::string> objects;
    string grammar;

    double period;
    bool isGrammarInitialized;
    Port iSpeakPort;
    Bottle iSpeakBottle;
    bool closing;
    Mutex mutex;
public:

    double getPeriod()
    {
        //returns the period of the module: a constant
        return period; //module periodicity (seconds)
    }

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {

        return !closing;
    }

    /*
    * Message handler. Just echo all received messages.
    */
    bool respond(const Bottle& command, Bottle& reply)
    {
        IH_Expand_vocab(iSpeakHelper_port, objects);

        yInfo()<<"Responding to command";
        //just echo back the command
        if (command.size() == 1 && command.get(0).asString()=="quit")
            return false;
        else if (command.size() == 1 && command.get(0).asString() == "listen") {
         reply = listen();
        } else if (command.size() == 2 && command.get(0).asString() == "talk") {

            reply.addInt(talk(command.get(1).asString()));
        }
        else{
            return false;
        }
        handlerPort.reply(reply);
        return true;
    }

    /*
    * Send grammar and objects to the speech recognition module
    */
    string IH_Expand_vocab(RpcClient& port, vector<string> objects) {
        if(!isGrammarInitialized){
            Bottle wb;
            Bottle reply;

            wb.addString("name");
            for (int i = 0; i < objects.size(); i++) {
                wb.addString(objects[i]);
            }
            port.write(wb,reply);
            string rep  =  reply.get(0).asString();

            if (rep == "ack") {
                isGrammarInitialized = true;
                for (int k = 0; k < objects.size(); k++) {
                    objects[k].clear();
                }
                for (int i=1;  i < reply.size()-1; i++) {
                    objects[i] = reply.get(i).asString();
                    yInfo() << "objects are: " << objects[i];
                }
            } else yInfo() <<"Was not able to set the new vocabulary: " << reply.get(0).asString();

            return rep;
        }
        return "";
    }

    /*
    * Return the string detected by the speech detector module
    */

    Bottle SM_Reco_Grammar(RpcClient& port, string gram) {
        Bottle wb;
        Bottle reply;
        wb.clear();
        wb.addString("recog");
        wb.addString("grammarSimple");
        wb.addString(gram);
        port.write(wb,reply);
        return reply;
    }
    /*
    *
    */
    bool talk(string phrase) {
        iSpeakBottle.clear();
        iSpeakBottle.addString(phrase);
        return iSpeakPort.write(iSpeakBottle);

    }
    /*
    *
    */
    Bottle listen() {
        Bottle recognized = SM_Reco_Grammar(speechRecog_port, grammar);
        return recognized;
    }

    /*
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {

        //-- initilization
        closing = false;
//        if(!iSpeakHelper_port.open("/iolHelper/rpc")) {
//            yError() << "Error opening helper rpc";
//            return false;
//        }

        if(!speechRecog_port.open("/speechRecognizer/rpc:o")) {
            yError() << "Error opening speech recognizer rpc";
            return false;
        }

        if(!handlerPort.open("/robot/voice_proc/rpc:o")) {
            yError() << "Error opening speech recognizer rpc";
            return false;
        }


        if(!iSpeakPort.open("/iSpeak:o")) {
            yError() << "Error opening iSpeak port";
            return false;
        }

        //-- defining objects and actions vocabularies
        objects = {"mug", "soda bottle"};

        //-- defining speech grammar for Menu
        grammar = "Where is the mug | Where is the soda bottle";

        isGrammarInitialized = false;
        period=1.0; //default value

        attach(handlerPort);

        //user resource finder to parse parameter --period

        if (rf.check("period"))
            period=rf.find("period").asDouble();

        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        closing = true;

        yInfo()<<"Interrupting your module, for port cleanup";
        return true;
    }

    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        yInfo()<<"Calling close function";
        mutex.lock();
        handlerPort.close();
        speechRecog_port.close();
        iSpeakHelper_port.close();
        iSpeakPort.close();
        mutex.unlock();
        return true;
    }
};

int main(int argc, char * argv[])
{
    Network yarp;

    NLPModule module;
    ResourceFinder rf;

    rf.configure(argc, argv);
    // rf.setVerbose(true);

    yInfo()<<"Configure module...";
    module.configure(rf);
    yInfo()<<"Start module...";
    module.runModule();

    yInfo()<<"Main returning...";
    return 0;
}


