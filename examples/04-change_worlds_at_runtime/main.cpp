// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Graphics.h"

#include <iostream>
#include <string>

using namespace std;

const string world_file_1 = "resources/world.urdf";
const string world_file_2 = "resources/world2.urdf";
const string robot_name_1 = "RBot";
const string robot_name_2 = "RBot2";

int main() {
    // load graphics scene
    auto graphics = new Sai2Graphics::Sai2Graphics(world_file_1);

    // setup robot joint positions
    Eigen::VectorXd q_robot1 = Eigen::VectorXd::Zero(1);
    Eigen::VectorXd q_robot2 = Eigen::VectorXd::Zero(1);

    unsigned long long counter = 0;
    int current_loaded_world = 0;

    // while window is open:
    while (graphics->isWindowOpen()) {
        // update robot position
        q_robot1(0) += 0.01;

        // update graphics rendering and window contents
        graphics->updateRobotGraphics(robot_name_1, q_robot1);
        if(current_loaded_world == 1) {
            q_robot2(0) -= 0.01;
            graphics->updateRobotGraphics(robot_name_2, q_robot2);
        }
        graphics->updateDisplayedWorld();

        // swap to second world
        if(counter % 700 == 350) {
            graphics->resetWorld(world_file_2);
            q_robot1.setZero();
            q_robot2.setZero();
            current_loaded_world = 1 - current_loaded_world;
        }
        // swap back to first world
        if((counter % 700 == 0) && (counter != 0)) {
            graphics->resetWorld(world_file_1);
            q_robot1.setZero();
            current_loaded_world = 1 - current_loaded_world;
        }
        counter++;
    }

    return 0;
}