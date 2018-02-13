
/**
* state_machine
*
* IDL Interface to \ref vino-blue module.
*/
service state_machine
{

    /**
     * Starts the state machine.
     * @param
     * @return true/false on success/failure.
     */
    bool execute(1:bool start);


    /**
     * Quit the module.
     * @return true/false on success/failure
     */
    bool quit();
}
