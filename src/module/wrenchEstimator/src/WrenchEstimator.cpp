#include "WrenchEstimator.h"

bool WrenchEstimator::readContactForce()
{
    if (m_rightHand)
    {
        cartesianWrench = rightWrenchInputPort.read(true);
    }
    else
    {
        cartesianWrench = leftWrenchInputPort.read(true);
    }
    if (cartesianWrench == nullptr)
    {
        yWarning() << "WrenchEstimator: Could not read contact force";
        return false;
    }

    contactForce(0) = (*cartesianWrench)[0];
    contactForce(1) = (*cartesianWrench)[1];
    contactForce(2) = (*cartesianWrench)[2];

    // Calibrate for offsets on first call
    if(m_forceThreshold_x < 0.1 && m_forceThreshold_y  < 0.1 && m_forceThreshold_z < 0.1){
        m_forceThreshold_x = contactForce(0) ;
        m_forceThreshold_y = contactForce(1);
        m_forceThreshold_z = contactForce(2);
        yInfo()<<"Updated thresholds";
        return true;
    }

    // remove offsets
    contactForce(0) -= m_forceThreshold_x ;
    contactForce(1) -=  m_forceThreshold_y ;
    contactForce(2) -=  m_forceThreshold_z ;

    yInfo() << "Force norm: " << yarp::math::norm(contactForce);
    yInfo() << "Force: " << contactForce(0) << " " << contactForce(1) << " "<< contactForce(2);
    if (yarp::math::norm(contactForce) < 1)
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


bool WrenchEstimator::estimateForceDirection(bool isRight = false)
{
    if (yarp::math::norm(contactForce) > 1)
    {
    // make sure magnitude of contact force is greater than 0
        yInfo()<<"Value of dot :" << yarp::math::dot(zVector, contactForce);
            if ( yarp::math::dot(zVector, contactForce) < 0)
            {
                m_forceFeedback = NEGATIVE_FEEDBACK;
                return true; // && isRight);
            }
            else if ( yarp::math::dot(zVector, contactForce) > 0)
            {
                m_forceFeedback = POSITIVE_FEEDBACK;
                return false; // && isRight);
            }
            else
            {
                yWarning() << "WrenchEstimator: Hey human! please touch against the palm.";
            }

    }
    return false;
}


// RFModule related methods

double WrenchEstimator::getPeriod() { return 0.01; }

bool WrenchEstimator::updateModule()
{
    readContactForce();

   return true;
}

bool WrenchEstimator::forceUpdate(){

    readContactForce();
    getHandPose();
    estimateForceDirection(false);
    yarp::os::Bottle& state = touchStateOuputPort.prepare();
    state.clear();
    state.addInt(m_forceFeedback);
    touchStateOuputPort.write();
    return true;
}

bool WrenchEstimator::respond(const yarp::os::Bottle& command, yarp::os::Bottle& response)
{
    m_forceFeedback = NO_FEEDBACK;
    // calibrate
    readContactForce();

    std::string cmd = command.get(0).asString();
    // rpc call
    if (cmd == "detectForces")
    {
        m_run = true;
        double t0 = yarp::os::Time::now();
        double timeout = 0.0;
        while(m_forceFeedback == FORCE_FEEDBACK::NO_FEEDBACK && timeout < 2)
        {
            forceUpdate();
            timeout += yarp::os::Time::now() - t0;
        }
        
        response.addInt(m_forceFeedback);
        stateMachineHandlerPort.reply(response);
    }

    m_run = false;

}

bool WrenchEstimator::configure(yarp::os::ResourceFinder &rf)
{
    bool ok = false;

    // open ports
    ok = rightWrenchInputPort.open("/wrench-estimator/right_arm/cartesianEndEffectorWrench:i");
    ok = ok && leftWrenchInputPort.open("/wrench-estimator/left_arm/cartesianEndEffectorWrench:i");
    ok = ok && touchStateOuputPort.open("/robot/forceFeedback:o");

    ok = ok && stateMachineHandlerPort.open("/robot/force_feedback/rpc");

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not open wrench input ports";
        return false;
    }

    // open cartesian interface
    yarp::os::Property optArmRight, optArmLeft;
    optArmRight.put("device", "cartesiancontrollerclient");
    optArmRight.put("remote", "/icub/cartesianController/right_arm");
    optArmRight.put("local", "/wrenchEstimator/cartesianClient/right_arm");

    optArmLeft.put("device", "cartesiancontrollerclient");
    optArmLeft.put("remote", "/icub/cartesianController/left_arm");
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

    // attach rpc service port
    attach(stateMachineHandlerPort);

    zVector.resize(3, 0);
    contactForce.resize(3, 0);

    return true;
}

bool WrenchEstimator::close()
{
    leftWrenchInputPort.close();
    rightWrenchInputPort.close();
    touchStateOuputPort.close();
    stateMachineHandlerPort.close();
    handPose = nullptr;
    cartesianWrench = nullptr;
    iArmLeft = nullptr;
    iArmRight = nullptr;

    return true;
}
