#include "WrenchEstimator.h"

bool WrenchEstimator::readContactForce()
{

    /////////////////// To remove //////////////
    yarp::sig::Vector& v = forcePort.prepare();
    v.clear();
    v.resize(8, 0);
    v[0] = 10.0;

    forcePort.write();

    /////////////////// To remove //////////////


    cartesianWrench = rightWrenchInputPort.read(false);
    if (cartesianWrench == nullptr)
    {
        yWarning() << "WrenchEstimator: Could not read contact force";
        return false;
    }

    contactForce(0) = (*cartesianWrench)[0];
    contactForce(1) = (*cartesianWrench)[1];
    contactForce(2) = (*cartesianWrench)[2];

    if (yarp::math::norm(contactForce) < 0)
    {
        return false;
    }
    return true;
}

bool WrenchEstimator::getHandPose()
{
    yarp::sig::Vector W_o_h(3);
    yarp::sig::Vector W_R_h_axisAngle(4);
    if (m_rightHand)
    {
        iArmRight->getPose(W_o_h, W_R_h_axisAngle);
    }
    else
    {
        iArmLeft->getPose(W_o_h, W_R_h_axisAngle);
    }

    yarp::sig::Matrix W_R_h = yarp::math::axis2dcm(W_R_h_axisAngle);
    zVector(0) = W_R_h(2, 0);
    zVector(1) = W_R_h(2, 1);
    zVector(2) = W_R_h(2, 2);

    yInfo() << "Z Axis: " << zVector(0) << " " << zVector(1) << " "<< zVector(2);

    return true;
}

///////////////////// TO REMOVE ///////////////////

bool WrenchEstimator::readHandPose()
{

    /////////////////// To remove //////////////
    yarp::sig::Vector& v = writePort.prepare();
    v.clear();
    v.resize(6, 0);
    v[4] = M_PI/2;

    writePort.write();

    /////////////////// To remove //////////////

    handPose = rightHandPoseInputPort.read(false);
    if (handPose == nullptr)
    {
        yWarning() << "WrenchEstimator: Could not read hand pose";
        return false;
    }

    yarp::sig::Vector W_R_h_asrpy(3);

    // Orientation of hand frame with respect to world
    W_R_h_asrpy(0) = (*handPose)[3];
    W_R_h_asrpy(1) = (*handPose)[4];
    W_R_h_asrpy(2) = (*handPose)[5];

    yarp::sig::Matrix W_R_h = yarp::math::rpy2dcm(W_R_h_asrpy);

    // Get the third column for the force direction
    zVector(0) = W_R_h(2, 0);
    zVector(1) = W_R_h(2, 1);
    zVector(2) = W_R_h(2, 2);

    return true;
}

///////////////////// TO REMOVE ///////////////////


bool WrenchEstimator::estimateForceDirection(bool isRight = true)
{
    // make sure magnitude of contact force is greater than 0
    if (yarp::math::dot(zVector, contactForce) == 0)
    {
        yWarning() << "WrenchEstimator: Hey human! please touch against the palm.";
    }
    else if ( yarp::math::dot(zVector, contactForce) > 0)
    {
        return (false && isRight);
    }
    else
    {
        return (true && isRight);
    }
}


// RFModule related methods

double WrenchEstimator::getPeriod() { return m_period; }

bool WrenchEstimator::updateModule()
{
    ///////////////////// TO REMOVE ///////////////////
    bool proceed = true;
//    bool proceed = readHandPose();
//    if ( !proceed )
//    {
////        yWarning() << "WrenchEstimator: Could not read contact force";
//        //return false;
//    }

    ///////////////////// TO REMOVE ///////////////////
    yarp::os::Time::delay(0.1);

    proceed = proceed && readContactForce();
    if (!proceed)
    {
        yError() << "WrenchEstimator: Could not read contact force";
        //return false;
    }

    yarp::os::Time::delay(0.1);

    if (proceed)
    {
        if (getHandPose())
        {
            if (estimateForceDirection(m_rightHand))
            {
                // need to write to port
                yInfo() << "WrenchEstimator: I sense that humans like what I did";
                m_forceFeedback = FORCE_FEEDBACK::POSITIVE_FEEDBACK;
            }
            else
            {
                yInfo() << "WrenchEstimator: Oops I did something wrong ??";
                m_forceFeedback = FORCE_FEEDBACK::NEGATIVE_FEEDBACK;
            }
        }
    }
    else
    {
        m_forceFeedback = FORCE_FEEDBACK::NO_FEEDBACK;
    }

    yarp::os::Bottle& state = touchStateOuputPort.prepare();
    state.clear();
    state.addInt(m_forceFeedback);
    touchStateOuputPort.write();

    contactForce.zero();
    zVector.zero();
    return true;
}

bool WrenchEstimator::configure(yarp::os::ResourceFinder &rf)
{
    bool ok = false;

    ok = rightWrenchInputPort.open("/wrench-estimator/right_arm/cartesianEndEffectorWrench:i");
    ok = ok && rightHandPoseInputPort.open("/wrench-estimator/right_hand/pose:i");
    ok = ok && touchStateOuputPort.open("/robot/forceFeedback:o");

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not open wrench input ports";
        return false;
    }

    yarp::os::Property optArmRight, optArmLeft;
    optArmRight.put("device", "cartesiancontrollerclient");
    optArmRight.put("remote", "/icubSim/cartesianController/right_arm");
    optArmRight.put("local", "/wrenchEstimator/cartesianClient/right_arm");

    optArmLeft.put("device", "cartesiancontrollerclient");
    optArmLeft.put("remote", "/icubSim/cartesianController/left_arm");
    optArmLeft.put("local", "/wrenchEstimator/cartesianClient/left_arm");

    ok = drvArmRight.open(optArmRight);
    ok = ok && drvArmLeft.open(optArmLeft);

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not open driver";
        return false;
    }

    ok = drvArmRight.view(iArmRight);
    ok = ok && drvArmLeft.view(iArmLeft);

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not view cartesian controllers";
        return false;
    }


    zVector.resize(3, 0);
    contactForce.resize(3, 0);

/////////////////// To remove //////////////
    ok = writePort.open("/pose");
    ok = ok && forcePort.open("/force");
    if (!ok)
    {
        yError() << "failed";
        return false;
    }
/////////////////// To remove //////////////

    return true;
}

bool WrenchEstimator::close()
{
    rightHandPoseInputPort.close();
    rightWrenchInputPort.close();
    touchStateOuputPort.close();
    handPose = nullptr;
    cartesianWrench = nullptr;
    iArmLeft = nullptr;
    iArmRight = nullptr;

    return true;
}
