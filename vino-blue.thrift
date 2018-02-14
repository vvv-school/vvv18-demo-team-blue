
/**
* state_machine
*
* IDL Interface to \ref vino-blue module.
*/

struct Bottle{}
(
    yarp.name = "yarp::os::Bottle"
    yarp.includefile="yarp/os/Bottle.h"
)

service state_machine
{

    /**
     * Starts the state machine.
     * @param
     * @return true/false on success/failure.
     */
    bool execute();

    /**
     * Starts the init action, This resets the state of the robot and brings him back to home position.
     * @param
     * @return true/false on success/failure.
     */
    bool init();

    /**
     * Start listening for command from user.
     * @param
     * @return the object_id to be detected.
     */
    i32 listenCommand();

    /**
     * @brief detectObject Detects a given object
     * @param objectid
     * @return true/false on success/failure. It also populates a class variable to store detected object location.
     */
     bool detectObject(1:i32 object_id);


    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}

