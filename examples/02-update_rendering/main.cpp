// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of
// it is also shown using Chai3D.

#include <iostream>
#include <string>

#include "Sai2Graphics.h"

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_name = "RBot";
const string object_name = "Box";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics =
		new Sai2Graphics::Sai2Graphics(world_file, "sai2 world", true);

	// robot joint pose
	Eigen::VectorXd robot_q = graphics->getRobotJointPos(robot_name);

	Eigen::Vector3d object_pos = Eigen::Vector3d(0, 0, -1.5);
	Eigen::Matrix3d object_ori = Eigen::Matrix3d::Identity();

	unsigned long long counter = 0;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update robot position
		robot_q << (double)counter / 100.0;

		// update object position
		object_pos(1) = -0.4 * sin((double)counter / 100);
		object_ori *= AngleAxisd(1.0 / 100.0, Eigen::Vector3d::UnitX())
						  .toRotationMatrix();

		// update graphics robot and object poses in graphics and render
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateObjectGraphics(object_name, object_pos,
									   Eigen::Quaterniond(object_ori));
		graphics->updateDisplayedWorld();

		counter++;
	}

	return 0;
}
