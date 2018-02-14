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


/***************************************************/
class CtrlModule: public RFModule
{
protected:
    PolyDriver drvArml, drvArmr, drvGaze;
    ICartesianControl *iarml, *iarmr;
    IGazeControl      *igaze;

    int startup_context_id;

    RpcServer rpcPort;

    Mutex mutex;

    bool simulation;

    int gaze_int;
    int arml_int;
    int armr_int;

    Vector initial_arml_position, initial_armr_position;
    Vector initial_arml_orientation, initial_armr_orientation;

    Vector initial_gaze;

    /***************************************************/
/*    void fixate(const Vector &x)
    {
        // FILL IN THE CODE
        igaze->lookAtFixationPointSync(x);
        igaze->waitMotionDone();
    }
*/
    /***************************************************/
/*    void roll(const Vector &x, const Vector &o)
    {
        // stoping a bit after the ball
        // y axis
        Vector xb = x;
        xb[1] = xb[1] - 10;
        iarm->goToPoseSync(xb, o);
        iarm->waitMotionDone();
    }
*/
    /***************************************************/
/*    void look_down()
    {
        Time::delay(5.0);
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        if (!simulation)
            igaze->blockEyes(5.0);

        // FILL IN THE CODE
        Vector ang(3); // setting the angle to 10.0 deg of vergence
        ang[0] = 0; ang[1] = -30; ang[2] = 10.0;
        igaze->lookAtAbsAnglesSync(ang);
        igaze->waitMotionDone();
    }
*/
    /***************************************************/
/*    bool make_it_roll(const Vector &cogL, const Vector &cogR)
    {
        Vector x;
        if (simulation)
        {
            yInfo()<<"detected cogs = ("<<cogL.toString(0,0)<<") ("<<cogR.toString(0,0)<<")";
            x=retrieveTarget3D(cogL,cogR);
        }
        else if (!object.getLocation(x))
            return false;

        yInfo()<<"retrieved 3D point = ("<<x.toString(3,3)<<")";

        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientation();
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        approachTargetWithHand(x,o);
        yInfo()<<"approached";

        roll(x,o);
        yInfo()<<"roll!";

        return true;
    }
*/
    /***************************************************/
    void happy()
    {
        Time::delay(5.0);
        /* TODO : check if this is important to keep this on if we do some gaze movements */
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        if (!simulation)
            igaze->blockEyes(5.0);

        // Happy gaze
        Vector ang(3); // setting the angle to 10.0 deg of vergence
        //ang[0] = 0; ang[1] = -30; ang[2] = 10.0;
        igaze->lookAtAbsAnglesSync(ang);
        igaze->waitMotionDone();

        // Happy arm gesture

        // Happy face display
    }

    /***************************************************/
    void sad()
    {
        Time::delay(5.0);
        /* TODO : check if this is important if we do some gaze movements */
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        if (!simulation)
            igaze->blockEyes(5.0);

        // sad gaze
        Vector ang(3); // setting the angle to 10.0 deg of vergence
        //ang[0] = 0; ang[1] = -30; ang[2] = 10.0;
        igaze->lookAtAbsAnglesSync(ang);
        igaze->waitMotionDone();

        // sad arm gesture

        // sad face display
    }

    /***************************************************/
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

        // wait that the movements are achieved
        iarml->waitMotionDone();
        iarmr->waitMotionDone();
        igaze->waitMotionDone();
    }

public:
    /***************************************************/
    bool configure(ResourceFinder &rf)
    {
        yInfo() << "Behaviors:: configuration of the behavior module...";
        string robot=rf.check("robot",Value("icubSim")).asString();
        simulation=(robot=="icubSim");

        // setting up the arms controller
        Property optArml, optArmr;
        optArml.put("device","cartesiancontrollerclient");
        optArml.put("remote","/"+robot+"/cartesianController/left_arm");
        optArml.put("local","/cartesian_client/left_arm");
        optArmr.put("device","cartesiancontrollerclient");
        optArmr.put("remote","/"+robot+"/cartesianController/right_arm");
        optArmr.put("local","/cartesian_client/right_arm");

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
        optGaze.put("local","/tracker/gaze");

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

        //setting up the port
        rpcPort.open("/robot/behavior/rpc:i");
        attach(rpcPort);

        yInfo() << "Behaviors:: configuration of the behavior module... done!";

        return true;
    }

    /***************************************************/
    bool interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool close()
    {
        drvArml.close();
        drvArmr.close();
        drvGaze.close();
        rpcPort.close();
        return true;
    }

    /***************************************************/
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
    double getPeriod()
    {
        return 0.0;     // sync upon incoming images
    }

    /***************************************************/
    bool updateModule()
    {
        //
 /*       // get fresh images
        ImageOf<PixelRgb> *imgL=imgLPortIn.read();
        ImageOf<PixelRgb> *imgR=imgRPortIn.read();

        // interrupt sequence detected
        if ((imgL==NULL) || (imgR==NULL))
            return false;

        // compute the center-of-mass of pixels of our color
        mutex.lock();
        okL=getCOG(*imgL,cogL);
        okR=getCOG(*imgR,cogR);
        mutex.unlock();

        PixelRgb color;
        color.r=255; color.g=0; color.b=0;

        if (okL)
            draw::addCircle(*imgL,color,(int)cogL[0],(int)cogL[1],5);

        if (okR)
            draw::addCircle(*imgR,color,(int)cogR[0],(int)cogR[1],5);

        imgLPortOut.prepare()=*imgL;
        imgRPortOut.prepare()=*imgR;

        imgLPortOut.write();
        imgRPortOut.write();
*/

        return true;
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
