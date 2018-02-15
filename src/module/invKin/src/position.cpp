#include <cmath>
#include <iostream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl2.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>

#include <yarp/math/Math.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;


class CtrlModule: public RFModule
{
protected:
    PolyDriver         clientJointL;
    PolyDriver         clientJointR;
    PolyDriver         clientCart;
    IControlLimits2   *ilim;
    IEncoders         *ienc;
    IControlMode2     *imod;
    IPositionControl2 *ipos;
    ICartesianControl  *iarm;


    IControlLimits2   *ilim1;
    IEncoders         *ienc1;
    IControlMode2     *imod1;
    IPositionControl2 *ipos1;

    bool simulation;

    RpcServer rpc;
    int joint;
    int fingers;
    std::vector<int> jointList ={8,9,10,11,12,13,14,15};
    std::vector<int> finger_joint_values = {70,15,180,6,7,85,150,240};
    std::vector<int> hi5jointList ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    std::vector<int> highFive_joint_vals ={-73,40,-10,94,2,-10,0,58,30,4,4,4,6,11,4,4};

    bool highFive()
    {

        for (int i=0;i<hi5jointList.size();i++)
        {
            joint = hi5jointList[i];
            // retrieve joint bounds
            double min,max,range;
            ilim1->getLimits(joint,&min,&max);
            range=max-min;

            // retrieve current joint position
            double enc;
            ienc1->getEncoder(joint,&enc);

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
            imod1->setControlMode(joint,VOCAB_CM_POSITION);

            // set up the speed in [deg/s]
            ipos1->setRefSpeed(joint,60.0);

            // set up max acceleration in [deg/s^2]
            ipos1->setRefAcceleration(joint,100.0);

            // yield the actual movement
            yInfo()<<"Yielding new target: "<<target<<" [deg]";
            ipos1->positionMove(joint,highFive_joint_vals[i]);

            // wait (with timeout) until the movement is completed
            bool done=false;
            double t0=Time::now();
            int count =0;
            while (!done&&(Time::now()-t0<10))
            {
                yInfo()<<"Waiting...";
                Time::delay(0.1);   // release the quantum to avoid starving resources
                ipos1->checkMotionDone(&done);
                if (done)
                    count++;
            }

            if (count == hi5jointList.size())
            {
                yInfo()<<"High 5 completed";
                return true;
            }
            else
            {
                yWarning()<<" High 5 Timeout expired";
                return false;
            }
        }
    }








    bool point_to_target(Vector x)
    {

                std::vector<int> count;
                Vector I_pos(3);
                Vector I_o(4);
                Vector pointing_pos(3);

                 for (int i=0;i<jointList.size();i++)
                {
                    fingers = jointList[i];
                    double enc;
                    double min,max,range;
                    ilim->getLimits(fingers,&min,&max);
                    ienc->getEncoder(fingers,&enc);
                    if (finger_joint_values[i] < min )
                {
                  yError() <<"Fist: joint " << i <<" below minimum threshold";
                  return false;
                }
                if (finger_joint_values[i] > max )
                {
                    yError() <<"Fist: joint " << i <<" above maximum threshold";
                    return false;
                }


                    ipos->setRefSpeed(fingers,60.0);

                    // set up max acceleration in [deg/s^2]
                    ipos->setRefAcceleration(fingers,100.0);

                    imod->setControlMode(fingers,VOCAB_CM_POSITION);
                    ipos->positionMove(fingers,finger_joint_values[i]);


                    bool done=false;

                        double t0=Time::now();
                        while (!done&&(Time::now()-t0<3))
                        {
                            yInfo()<<"Waiting...";
                            Time::delay(0.1);   // release the quantum to avoid starving resources
                            ipos->checkMotionDone(&done);
                            if (done)
                                count.push_back(i);
                        }


                }
                  if (count.size() == jointList.size())
                          {
                             yInfo()<<"Fist completed";
//                             return true;
                          }
                          else
                          {
                             yWarning()<<" Fist Timeout expired";
                             for (int j=0; j< count.size(); j++)
                                cout << count[j];
                           //  return false;
                          }

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

                Vector padded_x(4);
                padded_x.resize(4);
                padded_x[0] = x[0];
                padded_x[1] = x[1];
                padded_x[2] = x[2];
                padded_x[3] = 1.0;

                pointing_pos.resize(4);
                pointing_pos = T*padded_x;
                iarm->setTrajTime(1.0);
                iarm->goToPoseSync(pointing_pos,I_o);
                iarm->waitMotionDone();


                return true;

    }

public:
    virtual bool configure(ResourceFinder &rf)
    {



        // open a client interface to connect to the joint controller
        Property optJointL,optJointR, optCart;

        yInfo() << "KInematics:: configuration of the kinematics module...";
        string robot=rf.check("robot",Value("icubSim")).asString();
        simulation=(robot=="icubSim");

        optCart.put("device","cartesiancontrollerclient");
        optCart.put("remote","/"+robot+"/cartesianController/right_arm");
       // optCart.put("remote","/icubSim/cartesianController/right_arm");
        optCart.put("local","/cartesian_client/right_arm");

        optJointR.put("device","remote_controlboard");
        optJointR.put("remote","/"+robot+"/right_arm");
        //optJointR.put("remote","/icubSim/right_arm");
        optJointR.put("local","/robot/point_object/rpc/right_arm");

        optJointL.put("device","remote_controlboard");
       optJointL.put("remote","/"+robot+"/left_arm");
      // optJointL.put("remote","/icubSim/left_arm");
        optJointL.put("local","/robot/point_object/rpc/left_arm");

        if (!clientJointR.open(optJointR))
        {
            yError()<<"Unable to connect to /right_arm";
            return false;
        }
        if (!clientJointL.open(optJointL))
        {
            yError()<<"Unable to connect to /left_arm";
            return false;
        }
        if (!clientCart.open(optCart))
        {
            yError()<<"Unable to connect to /cart";
            return false;
        }


        // open views
        bool ok=true;
        ok=ok && clientJointL.view(ilim1);
        ok=ok && clientJointL.view(ienc1);
        ok=ok && clientJointL.view(imod1);
        ok=ok && clientJointL.view(ipos1);

        if (!ok)
        {
            yError()<<"Unable to open views left";
            return false;
        }

        // open views
        ok=true;
        ok=ok && clientJointR.view(ilim);
        ok=ok && clientJointR.view(ienc);
        ok=ok && clientJointR.view(imod);
        ok=ok && clientJointR.view(ipos);

        if (!ok)
        {
            yError()<<"Unable to open views right";
            return false;
        }


        // open views
        ok=true;
        ok=ok && clientCart.view(iarm);


        if (!ok)
        {
            yError()<<"Unable to open views cart";
            return false;
        }

        // elbow
        joint=3;

        // open rpc port
        rpc.open("/robot/point_object/rpc");

        // attach the callback respond()
        attach(rpc);

        return true;
    }

    virtual bool close()
    {
        rpc.close();
        clientJointR.close();
        clientJointL.close();
        clientCart.close();

        return true;
    }

    virtual bool respond(const Bottle &cmd, Bottle &reply)
    {
        reply.clear();
        if (cmd.get(0).asString()=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- point p0 p1 p2");
            reply.addString("- high");
            reply.addString("- quit");
        }
        else if(cmd.get(0).asString()=="point")
        {
            if(cmd.size()==4)
            {
                Vector target(3) ;
                target[0] = cmd.get(1).asDouble();
                target[1] = cmd.get(2).asDouble();
                target[2] = cmd.get(3).asDouble();
                point_to_target(target);
                reply.addInt(1);
                rpc.reply(reply);
                return true;
            }
            else reply.addInt(0);
        }
        else if (cmd.get(0).asString()=="high")
        {
            highFive();
            reply.addInt(1);
             rpc.reply(reply);
            return true;
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(cmd,reply);

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
