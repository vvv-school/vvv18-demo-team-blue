
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

struct Vector{}
(
    yarp.name = "yarp::sig::Vector"
    yarp.includefile="yarp/sig/Vector.h"
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
    bool listenCommand();

    /**
     * @brief detectObject Detects a given object
     * @param objectid
     * @return true/false on success/failure. It also populates a class variable to store detected object location.
     */
     bool detectObject(1:i32 object_id);

    /**
     * @brief pointAtObject calls the rpc to make the robot point at the given 3D point
     * @param position  a 3D point in cartesian coordinates in torso frame
     * @return  true/false on success/failure.
     */
     bool pointAtObject(1:Vector position);

    /**
     * @brief talk calls the rpc to make the robot talk the given text
     * @param text is the text to be spoken
     * @return  true/false on success/failure.
     */
     bool talk(1:string text);

    /**
     * @brief high_five calls the rpc to make the robot go in either high five or low five pose
     * @param val ture/false for high/low
     * @return  true/false on success/failure.
     */
     bool high_five(1:bool val);

    /**
     * @brief detect_forces calls the rpc to check force sensors on robot hand for a feedback from user
     * @return  -1 for no feedback, 0 for negative feedback, and 1 for positive feedback from user.
     */
     i32 detect_forces();

    /**
     * @brief display_expression calls rpc to make the robot display a given expression
     * @param expression it is a string with one of the values from happy, sad, home
     * @return  true/false on success/failure.
     */
     bool display_expression(1:string expression);


    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}

