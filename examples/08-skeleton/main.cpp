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

Eigen::VectorXd randomDrift(const Eigen::VectorXd& x, double sigma = 5e-2 * M_PI / 180) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, sigma);

    Eigen::VectorXd drift(x.size());

    for (int i = 0; i < x.size(); ++i)
        drift[i] = dist(gen);

    Eigen::VectorXd x_new = x + drift;
    x_new.head(6).setZero();
    return x_new;
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

    graphics->addMuscleTendonPathDisplay(muscle_file, robot_name);

    const double dt = 1. / 1000;

	// while window is open:
	while (graphics->isWindowOpen()) {

        // random drifting
        // robot->setQ(randomDrift(robot->q()));
        graphics->updateRobotGraphics(robot_name, robot->q());
        graphics->updateMuscleTendonPathDisplay();

		// update graphics the rendering and the window display.
		// this automatically waits for the correct amount of time
		graphics->renderGraphicsWorld();
	}

	return 0;
}
