// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;



class CtrlModule: public RFModule
{
protected:
    PolyDriver         clienJoint;
    IControlLimits2   *ilim;
    IEncoders         *ienc;
    IControlMode2     *imod;
    IPositionControl2 *ipos;
    ICartesianControl *iarm;

    RpcServer rpc;
    int joint;
    int jointEnc;
    std::vector<int> jointList ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    //std::vector<int> highFive_joint_vals ={-77,35,0,62,61,-28,-11,42,17,13,17,10,11,10,11,11};    
    //std::vector<int> highLow_joint_vals ={-30,20,18,55,-80,-24,-1,41,18,13,17,10,11,10,11,11};
    
    //void pointToObject(const Vector &x, const Vector &o)
    void pointToObject(Vector &x)
    {
        Vector I_pos;
        Vector I_o;
        Vector pointing_pos;
        /*//set offsets to avoid collision with the target
        double x_offset = 0.06;
        double y_offset = 0.01;
        double z_offset = 0.01;*/
        
        iarm->getPose(3,I_pos,I_o);
        Matrix R(3,3);
        Matrix T(4,4);
        R = axis2dcm(I_o);
        
        for(int i=0; i<2; i++)
        {
            for(int j=0; j<2; j++)
            {
                T[i][j] = R[i][j];
            }
        }
        
        for(int i=0; i<2; i++)
        {
            T[i][3] = I_pos[i];
        }
        
        T[3][0] = 0.0;
        T[3][1] = 0.0;
        T[3][2] = 0.0;
        T[3][3] = 1.0;
        
        Vector padded_x;
        padded_x.resize(4);
        padded_x.resize(4);
        padded_x[0] = x[0];
        padded_x[1] = x[1];
        padded_x[2] = x[2];
        padded_x[3] = 1.0;
        
        pointing_pos.resize(4);
        pointing_pos = T*x;
        
        
        //prepare the new arm position
        /*Vector new_x;
        new_x.resize(3);
        new_x[0] = x[0] + x_offset;
        new_x[1] = x[1] + y_offset;
        new_x[2] = x[2] + z_offset;*/
        
        //go to position
        iarm->setTrajTime(1.0);
        Vector xd, od, qd;
        if(iarm->askForPose(pointing_pos, I_o, xd, od, qd)){
            xd[1] = xd[1] - 0.1;
            iarm->goToPoseSync(pointing_pos,I_o);
            iarm->waitMotionDone();
            yInfo() <<"Pointing at the object!";
        }
        //iarm->goToPoseSync(pointing_pos,I_o);
        //iarm->waitMotionDone();
        //return true;



    }        

    /*bool highFive()
    {
    
        for (int i=0;i<jointList.size();i++)
        {
            joint = jointList[i];
            // retrieve joint bounds
            double min,max,range;
            ilim->getLimits(joint,&min,&max);
            range=max-min;

            // retrieve current joint position
            double enc;
            ienc->getEncoder(joint,&enc);

            // select target
            double target;
            if (highFive_joint_vals[i] < min )
            {
                yError() <<"Highfive: joint " << i <<" below minimum threshold";
                return false;
            }
            if (highFive_joint_vals[i] > max )
            {
                yError() <<"Highfive: joint " << i <<" above maximum threshold";
                return false;
             }   
              

            // set control mode
            imod->setControlMode(joint,VOCAB_CM_POSITION);

            // set up the speed in [deg/s]
            ipos->setRefSpeed(joint,60.0);

            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(joint,100.0);

            // yield the actual movement
            yInfo()<<"Yielding new target: "<<target<<" [deg]";
            ipos->positionMove(joint,highFive_joint_vals[i]);

            // wait (with timeout) until the movement is completed
            bool done=false;
            double t0=Time::now();
            while (!done&&(Time::now()-t0<10))
            {
                yInfo()<<"Waiting...";
                Time::delay(0.1);   // release the quantum to avoid starving resources
                ipos->checkMotionDone(&done);
            }

            if (done)
            {
                yInfo()<<"Highfive completed";
                return true;
            }
            else
            {
                yWarning()<<"Timeout expired";
                return false;  
            }
        }
    }*/


    /*bool highLow()
    {
    
        for (int i=0;i<jointList.size();i++)
        {
            joint = jointList[i];
            // retrieve joint bounds
            double min,max,range;
            ilim->getLimits(joint,&min,&max);
            if (!ilim)
            {
                yError() << "Highlow: Can't get joint limits!";
                return false;
            }


            // retrieve current joint position
            double enc;
            ienc->getEncoder(joint,&enc);
            if (!ienc)
            {
                yError() << "Can't get encoder values!";
                return false;
            }

            // select target
            double target;
            if (highLow_joint_vals[i] < min )
            {
                yError() <<"Highlow: joint " << i <<" below minimum threshold";
                return false;
            }
            if (highLow_joint_vals[i] > max )
            {
                yError() <<"Highlow: joint " << i <<" above maximum threshold";
                return false;
             }   
              

            // set control mode
            imod->setControlMode(joint,VOCAB_CM_POSITION);
            if (!imod)
            {
                yError() << "Highlow: Can't set control mode for joint " << i;
                return false;
            }
            

            // set up the speed in [deg/s]
            ipos->setRefSpeed(joint,60.0);
            

            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(joint,100.0);
           
            // yield the actual movement
            ipos->positionMove(joint,highLow_joint_vals[i]);

            // wait (with timeout) until the movement is completed
            bool done=false;
            double t0=Time::now();
            while (!done&&(Time::now()-t0<10))
            {
                yInfo()<<"Waiting...";
                Time::delay(0.1);   // release the quantum to avoid starving resources
                ipos->checkMotionDone(&done);
            }
        
            if (done)
            {
                yInfo()<<"Highlow completed";
                return true;
            }
            else
            {
                yWarning()<<"Timeout expired";
                return false;
            }
                
        }   
    }*/



public:
    virtual bool configure(ResourceFinder &rf)
    {
        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/icubSim/right_arm");
        optJoint.put("local","/position/right_arm");

        if (!clienJoint.open(optJoint))
        {
            yError()<<"Unable to connect to /icubSim/right_arm";
            return false;
        }

        // open views
        bool ok=true;
        ok=ok && clienJoint.view(ilim);
        ok=ok && clienJoint.view(ienc);
        ok=ok && clienJoint.view(imod);
        ok=ok && clienJoint.view(ipos);

        if (!ok)
        {
            yError()<<"Unable to open views";
            return false;
        }

        // elbow
            jointEnc=3;
        

        // open rpc port
        rpc.open("/position");

        // attach the callback respond()
        attach(rpc);

        return true;
    }

    virtual bool close()
    {
        rpc.close();
        clienJoint.close();

        return true;
    }

    virtual bool respond(const Bottle &cmd, Bottle &reply)
    {
        double x = cmd.get(0).asDouble();
        double y = cmd.get(1).asDouble();
        double z = cmd.get(2).asDouble();
        Vector pos;
        pos.resize(3);
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
        
        /*if(!pointToObject(pos))
        {
            yError() << "Could not point to object!";
        } else {
            reply.addString("Pointing to object!");
        }*/
        pointToObject(pos);
        
       //////// change to bool and checkpointToObject(xyz);
       
        
        /*if (cmd.get(0).asString()=="high")
        {
            if (highFive())
                reply.addString("High fived!");            
            else
                reply.addString("Could not high five!");            
        }
        else if (cmd.get(0).asString()=="low")
        {
            if (highLow())
                reply.addString("High lowed!");            
            else
                reply.addString("Could not high low!");            

        }*/

        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        return true;
    }
};



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
