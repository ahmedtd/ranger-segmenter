
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <armadillo>

#include <libplayerc++/playerc++.h>
using PlayerCc::PlayerClient;
using PlayerCc::RangerProxy;

int main(int argc, char **argv)
{
    PlayerClient client("localhost");
    RangerProxy  ranger(&client, 0);

    // Perform a preliminary fetch from the robot.
    client.Read();
    
    // Check the number of elements in the ranger
    // We only support laser-scanners (single-element, fan-of-readings devices)
    if(ranger.GetElementCount() != 1)
    {
        cerr << "This is not a laser rangefinder." << endl;
    }

    // Extract the scan parameters
    unsigned int num_readings = ranger.GetRangeCount();
    double angle_min          = ranger.GetMinAngle();
    double angle_max          = ranger.GetMaxAngle();
    double angle_delta        = (angle_max - angle_min) / num_readings;

    cout << angle_min << " " << angle_max << endl;

    while(true)
    {
        // Fetch updates from the robot
        client.Read();
        
        
    }

    return 0;
}
