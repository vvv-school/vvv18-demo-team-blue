#include "WrenchEstimator.h"

bool WrenchEstimator::readContactForce()
{
    contactForce.push_back(1);
    contactForce.push_back(1);
    contactForce.push_back(1);

    if (yarp::math::norm(contactForce) < 0)
    {
        return false;
    }
    return true;
}

bool WrenchEstimator::readHandPose()
{
    // FILL ME LATER - to be changed
    zVector.push_back(-1.0);
    zVector.push_back(0.0);
    zVector.push_back(1.0);
    return true;
}

bool WrenchEstimator::estimateForceDirection()
{
    // make sure magnitude of contact force is greater than 0
    if (yarp::math::dot(zVector, contactForce) == 0)
    {
        yWarning() << "WrenchEstimator: Hey human! please touch against the palm.";
    }
    else if ( yarp::math::dot(zVector, contactForce) > 0)
    {
        return false;
    }

    return true;
}


// RFModule related methods

double WrenchEstimator::getPeriod() { return m_period; }

bool WrenchEstimator::updateModule()
{
    if (!readContactForce())
    {
        yError() << "WrenchEstimator: Could not read contact force";
        return false;
    }

    if (!readHandPose())
    {
        yError() << "WrenchEstimator: Could not read contact force";
        return false;
    }

    if (estimateForceDirection())
    {
        // need to write to port
        yInfo() << "WrenchEstimator: I sense that humans like what I did";
    }
    else
    {
        yInfo() << "WrenchEstimator: I sense that humans like what I did";
    }
    return true;
}

bool WrenchEstimator::configure(yarp::os::ResourceFinder &rf)
{
    bool ok = false;

    ok = rightWrenchInputPort.open("/wrench-estimator/right_arm/cartesianEndEffectorWrench:i");
    ok = ok && rightHandPoseInputPort.open("/wrench-estimator/right_hand/pose:i");

    if (!ok)
    {
        yError() << "WrenchEstimator: Could not open wrench input ports";
        return false;
    }
    return true;
}

bool WrenchEstimator::close()
{

    return true;
}
