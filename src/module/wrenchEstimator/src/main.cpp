#include "WrenchEstimator.h"
#include <yarp/os/Network.h>
#include <iostream>
int main(int argc, char **argv)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    yarp::os::Network yarp;


    if (!yarp::os::Network::checkNetwork(5.0))
    {
        std::cout << "Yarp network not found\n";
        return false;
    }

    ResourceFinder rf = ResourceFinder::getResourceFinderSingleton();
    rf.configure(argc, argv);
    WrenchEstimator estimator;
    estimator.setName("wrench-estimator");

    return estimator.runModule(rf);
}
