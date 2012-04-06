
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
using std::vector;

#include <armadillo>
using arma::mat;
using arma::vec;
using arma::eye;
using arma::math;
using arma::dot;

#include <libplayerc++/playerc++.h>
using PlayerCc::PlayerClient;
using PlayerCc::RangerProxy;

// Derived from Roumeliotis Bekey 2000

int main(int argc, char **argv)
{
    PlayerClient client("localhost");
    RangerProxy  ranger(&client, 0);

    // Perform a preliminary fetch from the robot.
    client.Read();

    // Pull configuration information from remote ranger to the local ranger
    // proxy.
    ranger.RequestConfigure();
    ranger.RequestGeom();
    
        // Check the number of elements in the ranger
    // We only support laser-scanners (single-element, fan-of-readings devices)
    if(ranger.GetElementCount() != 1)
    {
        cerr << "This is not a laser rangefinder." << endl;
    }

    // Extract the scan parameters
    unsigned int num_readings = ranger.GetRangeCount();
    double       angle_min    = ranger.GetMinAngle();
    double       angle_max    = ranger.GetMaxAngle();
    double       angle_delta  = (angle_max - angle_min) / num_readings;

    cout << "Found laser ranger with properties: " << endl;
    cout << "min angle: "    << angle_min << endl;
    cout << "max angle: "    << angle_max << endl;
    cout << "num readings: " << num_readings << endl;
    cout << "angle delta: "  << angle_delta << endl;
    
    // Set up the strict and loose model variances.
    // These are the expected variances of where the next point will lie on the
    // wall.
    mat model_var_strict = eye<double>(2);
    model_strict(0,0) = 0.1;
    model_strict(1,1) = (math::pi() / 180.0);
    
    mat model_var_loose = eye<double>(2);
    model_loose(0,0) = 0.3;
    model_loose(1,1) = (math::pi() / 180.0)*2;

    // The jacobian of the measurement selection function, h.  Since h selects
    // the range component of the current state, the jacobian is [1 0]
    mat jac_h(1,2);
    jac_h(0,0) = 1.0;
    jac_h(0,1) = 0.0;

    // Set up the measurement (sensor) variance. This is just the variance in
    // the range.
    double sensor_var = 0.01;

    // Turn on the laser
    ranger.SetPower(true);

    // Storage for the point sequence
    mat points(2, num_readings);

    // Storage for the delta sequence
    mat deltas(2, num_readings-1);

    // Storage for the cross product and dot product sequences
    mat crossps(1, num_readings-2);
    mat dotps(1, num_readings-2);

    while(true)
    {
        // Fetch updates from the robot
        client.Read();
        
        // Copy readings from the ranger proxy
        double angle = angle_min;
        for(int count = 0; count < num_readings; count++)
        {
            points(0, count) = ranger[count] * cos(angle);
            points(1, count) = ranger[count] * sin(angle);
            
            angle += angle_delta;
        }

        // Compute deltas
        for(int count = 1; count < num_readings; count++)
        {
            deltas.col(count-1) = points.col(count) - points.col(count-1);
        }

        // Compute cross products and dot products
        for(int count = 2; count < num_readings; count++)
        {
            crossps(0, count-2) = deltas(0,count-2)*deltas(1, count-1)
                                  - deltas(0, count-1)*deltas(1, count-2);
            
            dotps(0, count-2) = dot(deltas.col(count-2), deltas.col(count-1));
        }

        // Bootstrap output
        angle = angle_min;
        for(int count = 2; count < num_readings; count++)
        {
            cout << angle << " " << crossps(0, count-2) << endl;
            angle += angle_delta;
        }

        cout << endl;

        break;
    }

    return 0;
}
