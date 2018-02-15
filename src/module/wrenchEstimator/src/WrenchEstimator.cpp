#include "WrenchEstimator.h"

bool WrenchEstimator::readContactForce()
{

    if (m_rightHand)
    {
        cartesianWrench = rightWrenchInputPort.read(false);
    }
    else
    {
        cartesianWrench = leftWrenchInputPort.read(false);
    }
    if (cartesianWrench == nullptr)
    {
        yWarning() << "WrenchEstimator: Could not read contact force";
        return false;
    }

    contactForce(0) = (*cartesianWrench)[0];
    contactForce(1) = (*cartesianWrench)[1];
    contactForce(2) = (*cartesianWrench)[2];

    yInfo() << "Force norm: " << yarp::math::norm(contactForce);
    yInfo() << "Force: " << contactForce(0) << " " << contactForce(1) << " "<< contactForce(2);
    if (yarp::math::norm(contactForce) < m_forceThreshold)
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


bool WrenchEstimator::estimateForceDirection(bool isRight = true)
{
    if (yarp::math::norm(contactForce) > m_forceThreshold)
    {
    // make sure magnitude of contact force is greater than 0

            if ( yarp::math::dot(zVector, contactForce) < -1)
            {
                return (true && isRight);
            }
            else if ( yarp::math::dot(zVector, contactForce) > 1)
            {
                return (false && isRight);
            }
            else
            {
                yWarning() << "WrenchEstimator: Hey human! please touch against the palm.";
            }

    }
    return false;
}


// RFModule related methods

double WrenchEstimator::getPeriod() { return m_period; }

bool WrenchEstimator::updateModule()
{
    if (m_run)
    {
        bool proceed = true;

        proceed = proceed && readContactForce();
        if (!proceed)
        {
            yError() << "WrenchEstimator: Could not read contact force";
            //return false;
        }

        //yarp::os::Time::delay(0.1);

        if (proceed)
        {
            if (getHandPose())
            {
                if (estimateForceDirection(m_rightHand))
                {
                    // need to write to port
                    yInfo() << "WrenchEstimator: I sense that humans like what I did";
                    m_mutex.lock();
                    m_forceFeedback = FORCE_FEEDBACK::POSITIVE_FEEDBACK;
                    m_mutex.unlock();
                }
                else
                {
                    yInfo() << "WrenchEstimator: Oops I did something wrong ??";
                    m_mutex.lock();
                    m_forceFeedback = FORCE_FEEDBACK::NEGATIVE_FEEDBACK;
                    m_mutex.unlock();
                }
            }
        }
        else
        {
            m_mutex.lock();
            m_forceFeedback = FORCE_FEEDBACK::NO_FEEDBACK;
            m_mutex.unlock();
        }

        yarp::os::Bottle& state = touchStateOuputPort.prepare();
        state.clear();
        state.addInt(m_forceFeedback);
        touchStateOuputPort.write();
    }
    contactForce.zero();
    zVector.zero();
    return true;
}

bool WrenchEstimator::respond(const yarp::os::Bottle& command, yarp::os::Bottle& response)
{
    std::string cmd = command.get(0).asString();
    if (cmd == "detectForces")
    {
        m_mutex.lock();
        m_run = true;
        m_mutex.unlock();

        double t0 = yarp::os::Time::now();
        double timeout = 0.0;
        while(m_forceFeedback == FORCE_FEEDBACK::NO_FEEDBACK || timeout < 2)
        {
            timeout += yarp::os::Time::now() - t0;
            yInfo() << "Timeout: " << timeout;
        }

        response.addInt(m_forceFeedback);
        stateMachineHandlerPort.reply(response);
    }
    m_mutex.lock();
    m_run = false;
    m_mutex.unlock();

}

bool WrenchEstimator::configure(yarp::os::ResourceFinder &rf)
{
    bool ok = false;

    ok = rightWrenchInputPort.open("/wrench-estimator/right_arm/cartesianEndEffectorWrench:i");
    ok = ok && leftWrenchInputPort.open("/wrench-estimator/left_arm/cartesianEndEffectorWrench:i");
    ok = ok && touchStateOuputPort.open("/robot/forceFeedback:o");

    ok = ok && stateMachineHandlerPort.open("/robot/force_feedback/rpc");

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not open wrench input ports";
        return false;
    }

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
