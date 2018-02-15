#ifndef WRENCHESTIMATOR_H
#define WRENCHESTIMATOR_H

#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/os/Network.h>
#include <yarp/os/RpcServer.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Mutex.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/os/Vocab.h>

#include <cmath>

/**
 * @brief The FORCE_FEEDBACK enum
 * To get the direction of the external force
 */
enum FORCE_FEEDBACK
{
    NO_FEEDBACK=-1,
    NEGATIVE_FEEDBACK,
    POSITIVE_FEEDBACK
};

class WrenchEstimator : public yarp::os::RFModule
{
    // Port to read the cartesian end effector wrenches
    yarp::os::BufferedPort<yarp::sig::Vector> leftWrenchInputPort;
    yarp::os::BufferedPort<yarp::sig::Vector> rightWrenchInputPort;

    // Port to force feedback data
    yarp::os::BufferedPort<yarp::os::Bottle> touchStateOuputPort;

    // RPC port to talk to the state machine
    yarp::os::RpcServer stateMachineHandlerPort;

    // External wrench acting on the hand
    yarp::sig::Vector m_handWrench;
    // Pose of te hand
    yarp::sig::Vector m_handPose; // x y z r p y

    // Direction of force vector
    yarp::sig::Vector zVector;

    // Vector to read pose of the hand from cartesian controller
    yarp::sig::Vector* handPose{nullptr};

    // Vector to read wrench from whole body dynamics
    yarp::sig::Vector* cartesianWrench{nullptr};
    // contact force vector
    yarp::sig::Vector contactForce;

    // Cartesian control interface
    yarp::dev::PolyDriver drvArmRight, drvArmLeft;
    yarp::dev::ICartesianControl *iArmRight{nullptr}, *iArmLeft{nullptr};

    // Read methods
    bool readContactForce();

    /**
     * @brief getHandPose
     * Gets pose of the hand from the cartesian controller and gets the
     * z-vector of the hand
     * @return true if successful
     */
    bool getHandPose();

    /**
     * @brief estimateForceDirection
     * Performs a dot product of the z axis vector of palm and force acting on it
     * @return true is good, bad otherwise
     */
    bool estimateForceDirection(bool isRight);


    bool forceUpdate();

    double m_period = 0.1;

    // which feedback
    int m_forceFeedback = -1;

    // which hand
    bool m_rightHand = false;

    // run only after state machine flag
    bool m_run = false;

    bool isClosing = false;

    // threshold to account for zmobie force values
    double m_forceThreshold_x = 0.0;
    double m_forceThreshold_y = 0.0;
    double m_forceThreshold_z = 0.0;
    yarp::os::Mutex m_mutex;

public:

    virtual double getPeriod ();
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
    virtual bool interruptModule();
};

#endif

