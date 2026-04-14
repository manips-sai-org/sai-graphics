// This example application loads a URDF world file and displays a simple robot
// that does not move

#include <iostream>
#include <filesystem>
#include <string>
#include <SaiModel.h>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/08-skeleton/world.urdf";
const string robot_file = 
    string(EXAMPLES_FOLDER) + "/08-skeleton/models/rajagopal_model/Rajagopal2015.urdf";
const string muscle_file = 
    string(EXAMPLES_FOLDER) + "/08-skeleton/muscles_full.xml";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

    const std::string robot_name = "Rajagopal2015";

	// load graphics scene
	auto graphics =
		new SaiGraphics::SaiGraphics(world_file, "sai world", true);

    auto robot = graphics->getRobot(robot_name);
    robot->setQ(VectorXd::Zero(robot->dof()));
    robot->updateKinematics();

    // graphics->showMovableJointFrames(true, "Rajagopal2015", 0.20, true);
    graphics->showJointPositionSliders(true, robot_name);

    graphics->addMuscleTendonPathDisplay(muscle_file, robot_name);
    auto last_muscle_write_time = std::filesystem::last_write_time(muscle_file);

	// while window is open:
	while (graphics->isWindowOpen()) {
        const auto current_muscle_write_time =
            std::filesystem::last_write_time(muscle_file);
        if (current_muscle_write_time != last_muscle_write_time) {
            try {
                graphics->reloadMuscleTendonPathDisplay(muscle_file, robot_name);
                last_muscle_write_time = current_muscle_write_time;
                cout << "Reloaded muscle tendon paths from " << muscle_file << endl;
            } catch (const std::exception& e) {
                cerr << "Failed to reload muscle tendon paths: " << e.what() << endl;
            }
        }
            
        graphics->updateRobotGraphics(robot_name, robot->q());
        graphics->updateMuscleTendonPathDisplay();

		// update graphics the rendering and the window display.
		// this automatically waits for the correct amount of time
		graphics->renderGraphicsWorld();
	}

	return 0;
}
