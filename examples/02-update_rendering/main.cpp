// This example application loads a URDF world file with a pendulum and a cube,
// and updates their position continuously

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

	// object position and orientation
	Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
	object_pose.translation() = Eigen::Vector3d(0, 0, -1.5);

	unsigned long long counter = 0;

	cout << endl
		 << "This example parses a world file containing a pedulum and a cube, "
			"and updates their positions"
		 << endl;

	// while window is open:
	while (graphics->isWindowOpen()) {
		// update robot position
		robot_q << (double)counter / 100.0;

		// update object position
		object_pose.translation()(1) = -0.4 * sin((double)counter / 100);
		object_pose.linear() *=
			AngleAxisd(1.0 / 100.0, Eigen::Vector3d::UnitX())
				.toRotationMatrix();

		// update graphics robot and object poses in graphics and render
		graphics->updateRobotGraphics(robot_name, robot_q);
		graphics->updateObjectGraphics(object_name, object_pose);
		graphics->renderGraphicsWorld();

		counter++;
	}

	return 0;
}
