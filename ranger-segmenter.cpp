
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
using arma:: math;

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

    // Set up the measurement (sensor) variance. This is just the variance in
    // the range.
    double sensor_var = 0.01;

    

    // Turn on the laser
    ranger.SetPower(true);

    // Storage for the range sequence pulled from the ranger
    vector<double> ranges(num_readings);

    while(true)
    {
        // Fetch updates from the robot
        client.Read();
        
        // Copy readings from the ranger proxy
        ranges.clear();
        for(int count = 0; count < num_readings; count++)
        {
            ranges[count] = ranger[count];
        }

        // Set up initial estimates for the Kalman filters
        vec cur_state(2);
        cur_state(0) = ranges.front();
        cur_state(1) = angle_min;

        mat cur_var = eye<double>(2) * 100.0;

        // Step along the range readings with a kalman filter.
        for(int index = 0; index < ranges.size(); index++)
        {
            // Take some common functions
            double sin_theta_k     = sin(cur_state(1));
            double sin_theta_k_1   = sin(cur_state(1) + angle_delta);
            double sin_delta_theta = sin(angle_delta);
            
            // Take the jacobian of the transition function
            mat jac = eye<double>(2);
            jac(0,0) = sin_theta_k / sin_theta_k_1;
            jac(0,1) = sin_delta_theta / pow(sin_theta_k_1, 2) * cur_state(0);

            // Compute predicted state
            vec pred_state(2);
            pred_state(0) = sin_theta_k / sin_theta_k_1 * cur_state(0);
            pred_state(1) = cur_state(1) + angle_delta;

            mat pred_var = jac * cur_var * jac.t() + 
            
        }
    }

    return 0;
}
