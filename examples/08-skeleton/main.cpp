// This example application loads a URDF world file and displays a simple robot
// that does not move

#include <iostream>
#include <string>
#include <random>
#include <SaiModel.h>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/08-skeleton/world.urdf";
const string robot_file = 
    string(EXAMPLES_FOLDER) + "/08-skeleton/human.urdf";
const string muscle_file = 
    string(EXAMPLES_FOLDER) + "/08-skeleton/muscles_fixed.xml";

// Augmented to include a 'mean' parameter for non-zero mean drift
Eigen::VectorXd randomDrift(const Eigen::VectorXd& x, 
                            double mean = 1e-4, // Example default non-zero mean
                            double sigma = 5e-2 * M_PI / 180) {
    
    // Use static for the generator so it isn't re-seeded on every function call
    static std::random_device rd;
    static std::mt19937 gen(rd());
    
    // Define a normal distribution with the specified mean and standard deviation
    std::normal_distribution<double> dist(mean, sigma);
    
    // Create a drift vector of the same size as 'x', populated via the distribution
    Eigen::VectorXd drift = Eigen::VectorXd::NullaryExpr(x.size(), [&]() {
        return dist(gen);
    });
    
    // Return just the drift vector, or return 'x + drift' if you want the updated state
    return drift; 
}

// Sweeps joint positions between lower and upper limits over time 't'
Eigen::VectorXd sinusoidalSweep(double t, 
                                const Eigen::VectorXd& lower_limits, 
                                const Eigen::VectorXd& upper_limits, 
                                double frequency_hz = 0.5) {
    
    // Safety check to ensure dimension match
    if (lower_limits.size() != upper_limits.size()) {
        throw std::invalid_argument("Lower and upper limit vectors must be the same size.");
    }
    
    // Calculate the midpoint (offset) and amplitude vectors
    Eigen::VectorXd midpoint = (upper_limits + lower_limits) / 2.0;
    Eigen::VectorXd amplitude = (upper_limits - lower_limits) / 2.0;
    
    // Calculate the scalar sine value based on time and frequency
    double phase = 2.0 * M_PI * frequency_hz * t;
    double sine_val = std::sin(phase);
    
    // Broadcast the scalar sine value across the amplitude vector and add the midpoint
    return midpoint + amplitude * sine_val;
}

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

    const std::string robot_name = "human";

	// load graphics scene
	auto graphics =
		new SaiGraphics::SaiGraphics(world_file, "sai world", true);

    auto robot = graphics->getRobot(robot_name);
    robot->setQ(VectorXd::Zero(robot->dof()));
    robot->updateKinematics();

    auto joint_limits = robot->jointLimits();
    VectorXd q_lower(robot->dof()), q_upper(robot->dof());
    int i = 0;
    for (auto limit : joint_limits) {
        q_lower(i) = limit.position_lower;
        q_upper(i) = limit.position_upper;
        ++i;
    }

    graphics->addMuscleTendonPathDisplay(muscle_file, robot_name);

    auto start_time = std::chrono::steady_clock::now();

	// while window is open:
	while (graphics->isWindowOpen()) {
            
        // 2. Record the current time
        auto current_time = std::chrono::steady_clock::now();

        // 3. Calculate the elapsed time as a double in seconds
        std::chrono::duration<double> elapsed_seconds = current_time - start_time;
        
        // Now you can pass 't' to your sinusoidalSweep function!
        double t = elapsed_seconds.count();

        // sine sweep
        VectorXd q = sinusoidalSweep(t, q_lower, q_upper);
        q.head(6).setZero();
        q.setZero();
        robot->setQ(q);

        graphics->updateRobotGraphics(robot_name, robot->q());
        graphics->updateMuscleTendonPathDisplay();

		// update graphics the rendering and the window display.
		// this automatically waits for the correct amount of time
		graphics->renderGraphicsWorld();
	}

	return 0;
}
