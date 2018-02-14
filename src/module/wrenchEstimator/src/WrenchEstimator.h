#ifndef WRENCHESTIMATOR_H
#define WRENCHESTIMATOR_H

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <yarp/math/Math.h>

#include <cmath>


class WrenchEstimator : public yarp::os::RFModule
{

    yarp::os::BufferedPort<yarp::sig::Vector> rightHandPoseInputPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rightWrenchInputPort;

    yarp::sig::Vector m_rightHandWrench;
    yarp::sig::Vector m_rightHandPose; // x y z


    // delete when other module is ready
    yarp::sig::Vector zVector;
    yarp::sig::Vector contactForce;

    bool readContactForce();
    bool readHandPose();

    /**
     * @brief estimateForceDirection
     * Performs a dot product of the z axis vector of palm and force acting on it
     * @return true is good, bad otherwise
     */
    bool estimateForceDirection();

    double m_period;

    std::string m_hand;
public:

    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};

#endif

