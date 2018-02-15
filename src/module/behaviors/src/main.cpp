// -*- mode: C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Authors: Pauline Chevalier - <pauline.chevalier313@gmail.com>
//          Ugo Pattacini - <ugo.pattacini@iit.it>

#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "helpers.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#define FACE_HAPPY ("hap")
#define FACE_SAD ("sad")
#define FACE_SURPRISED ("sur")

/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArml, drvArmr, drvGaze;
    ICartesianControl *iarml, *iarmr;
    IGazeControl      *igaze;

    int startup_context_id;

    RpcServer rpcPort;
    Port portFace;

    Mutex mutex;

    bool simulation;
    bool isClosing;

    int gaze_int;
    int arml_int;
    int armr_int;

    Vector initial_arml_position, initial_armr_position;
    Vector initial_arml_orientation, initial_armr_orientation;

    Vector initial_gaze;
    
    /**
     * @brief setFace
     * @param type
     * Set a -type- facial expression on the iCub
     */
    void setFace(const string &type) {
        Bottle out;

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("mou"));
        out.addVocab(Vocab::encode(type.c_str()));
        portFace.write(out);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("leb"));
        out.addVocab(Vocab::encode(type.c_str()));
        portFace.write(out);

        out.clear();

        out.addVocab(Vocab::encode("set"));
        out.addVocab(Vocab::encode("reb"));
        out.addVocab(Vocab::encode(type.c_str()));
        portFace.write(out);
    }

    /***************************************************/
    /**
     * @brief happy
     * Animation for an happy gesture
     * head and facial movements
     */
    void happy()
    {
        Vector ang(3);
        Vector ang_1(3);
        ang[0] = 0; ang[1] = 10; ang[2] = 5.0;
        ang_1[0] = 5; ang_1[1] = 10; ang_1[2] = 5;

        // Surprised face display
        if (!simulation) {
            setFace(FACE_SURPRISED);
        }
        // Surpised head going back
        igaze->lookAtAbsAnglesSync(ang);
        igaze->waitMotionDone();

        // Happy face display
        if (!simulation)
            setFace(FACE_HAPPY);

        // Happy gaze
        for (int ii = 0; ii < 2; ii++) {
            ang[1] = ang_1[ii];
            igaze->lookAtAbsAnglesSync(ang);
            igaze->waitMotionDone();
        }

        // Happy arm gesture

        // getting back to home
        home();

    }

    /***************************************************/
    /**
     * @brief sad
     * Animation for an happy gesture
     * head and facial movements
     */
    void sad()
    {
        Vector ang(3);
        Vector ang_0(3);
        ang[0] = 0; ang[1] = -20; ang[2] = 5.0;
        ang_0[0] = 0; ang_0[1] = -5; ang_0[2] = 5; ang_0[3] = -5;

        // sad face display
        if (!simulation)
            setFace(FACE_SAD);

        // sad gaze
        for (int ii = 0; ii < 3; ii++) {
            ang[0] = ang_0[ii];
            igaze->lookAtAbsAnglesSync(ang);
            igaze->waitMotionDone();
        }

        // sad arm gesture

        // getting back to home
        home();
    }

    /***************************************************/
    /**
     * @brief home
     * Set back the robot to the home position
     * The home position is recorded at the configuration of the behavior module
     */
    void home()
    {
        // go back to the initail position of the robot
        iarml->setLimits(0, 0.0, 0.0);
        iarml->setLimits(1, 0.0, 0.0);
        iarml->setLimits(2, 0.0, 0.0);
        iarml->goToPoseSync(initial_arml_position, initial_arml_orientation);
        iarmr->setLimits(0, 0.0, 0.0);
        iarmr->setLimits(1, 0.0, 0.0);
        iarmr->setLimits(2, 0.0, 0.0);
        iarmr->goToPoseSync(initial_armr_position, initial_armr_orientation);
        igaze->lookAtAbsAngles(initial_gaze);

        if (!simulation)
            setFace(FACE_HAPPY);

        // wait that the movements are achieved
        iarml->waitMotionDone();
        iarmr->waitMotionDone();
        igaze->waitMotionDone();
    }

public:
    /***************************************************/
    /**
     * @brief configure
     * @param rf
     * @return
     */
    bool configure(ResourceFinder &rf)
    {
        yInfo() << "Behaviors:: configuration of the behaviors module...";
        string robot=rf.check("robot",Value("icubSim")).asString();
        simulation=(robot=="icubSim");
        isClosing=false;
        // setting up the arms controller
        Property optArml, optArmr;
        optArml.put("device","cartesiancontrollerclient");
        optArml.put("remote","/"+robot+"/cartesianController/left_arm");
        optArml.put("local","/behaviors/cartesian_client/left_arm");
        optArmr.put("device","cartesiancontrollerclient");
        optArmr.put("remote","/"+robot+"/cartesianController/right_arm");
        optArmr.put("local","/behaviors/cartesian_client/right_arm");

        // let's give the controller some time to warm up
        bool ok_arms=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0) {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArml.open(optArml) && drvArmr.open(optArmr)) {
                ok_arms=true;
                break;
            }
            Time::delay(1.0);
        }
        if (!ok_arms) {
            yError()<<"Unable to open the Cartesian Controller";
            return false;
        }

        // setting up the gaze controller
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/behaviors/tracker/gaze");

        bool ok_gaze=false;
        t0=Time::now();
        while (Time::now()-t0<10.0) {
            if (drvGaze.open(optGaze)) {
                ok_gaze=true;
                break;
            }
            Time::delay(1.0);
        }

        // open the views
        if (drvArmr.isValid()) {
            drvArmr.view(iarmr);
        }
        if (drvArml.isValid()) {
            drvArml.view(iarml);
        }
        if (drvGaze.isValid()) {
           drvGaze.view(igaze);
        }

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the tracking mode, the neck limits and so on.
        iarml->storeContext(&arml_int);
        iarmr->storeContext(&armr_int);
        igaze->storeContext(&startup_context_id);

        // set trajectory time
        igaze->setNeckTrajTime(0.6);
        igaze->setEyesTrajTime(0.4);

        // get inital positions
        iarml->getPose(initial_arml_position, initial_arml_orientation);
        iarmr->getPose(initial_armr_position, initial_armr_orientation);
        igaze->getAngles(initial_gaze);

        // setting up some limits to the arms
        // to not reach difficult/dangerous positions
        Vector  newDof_l, curDof_l, newDof_r, curDof_r;
        iarml->getDOF(curDof_l);
        iarml->getDOF(curDof_r);
        newDof_l=curDof_l;
        newDof_r=curDof_r;

        newDof_l[0]=1; newDof_l[1]=0; newDof_l[2]=1;
        newDof_r[0]=1; newDof_r[1]=0; newDof_r[2]=1;

        iarml->setDOF(newDof_l,curDof_l);
        iarmr->setDOF(newDof_r,curDof_r);

        double min,max;
        iarml->getLimits(0,&min,&max);
        iarml->setLimits(0,min,30.0);

        iarmr->getLimits(0,&min,&max);
        iarmr->setLimits(0,min,30.0);

        // setting up the facial expressions :
        // sending commands to the face command RPC server
        if(!simulation){
            portFace.open("/robot/behavior/emotions:o");
            attach(portFace);
        }

        //setting up the input port of the module
        rpcPort.open("/robot/behavior/rpc:i");
        yInfo() << "Behaviors:: port open at </robot/behavior/rpc:i>";

        attach(rpcPort);

        yInfo() << "Behaviors:: configuration of the behaviors module... done!";

        return true;
    }

    /***************************************************/
    /**
     * @brief interruptModule
     * @return
     */
    bool interruptModule()
    {
        isClosing=true;
        return true;
    }

    /***************************************************/
    /**
     * @brief close
     * @return
     */
    bool close()
    {
        drvArml.close();
        drvArmr.close();
        drvGaze.close();
        rpcPort.close();
        if (!simulation) {
            setFace(FACE_HAPPY);
            portFace.interrupt();
            portFace.close();
        }
        return true;
    }

    /***************************************************/
    /**
     * @brief respond
     * @param command
     * @param reply
     * @return
     */
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- happy");
            reply.addString("- sad");
            reply.addString("- home");
            reply.addString("- quit");
        }
        else if (cmd=="happy")
        {
            happy();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm happy!");
        }
        else if (cmd=="sad")
        {
            sad();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Oh no, I am sad...");
        }
        else if (cmd=="home")
        {
            home();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("I've got the hard work done! Gone home.");
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);

        return true;
    }

    /***************************************************/
    /**
     * @brief getPeriod
     * @return
     */
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    /**
     * @brief updateModule
     * @return
     */
    bool updateModule()
    {
        yarp::os::Time::delay(1.0);
        return !isClosing;
    }
};


/***************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    CtrlModule mod;
    return mod.runModule(rf);
}
