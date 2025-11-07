// This example shows we can right click on the UI window and drag the mouse
// to generate forces. In this example, we show the line corresponding to the
// forces and get the corresponding joint torques for the robot, to display in
// terminal.

#include <iostream>
#include <string>

#include "SaiGraphics.h"

using namespace std;

const string world_file =
	string(EXAMPLES_FOLDER) + "/05-apply_ui_force/world.urdf";
const string robot_name = "RBot";
const string object_name = "Box";

int main()
{
	cout << "Loading URDF world model file: " << world_file << endl;

	// load graphics scene
	auto graphics = std::make_shared<SaiGraphics::SaiGraphics>(world_file);
	Eigen::VectorXd robot_q = graphics->getRobotJointPos(robot_name);
	Eigen::Affine3d object_pose = graphics->getObjectPose(object_name);

	// set up ui force interaction
	graphics->addUIForceInteraction(robot_name);
	Eigen::VectorXd ui_interaction_torques_robot;
	graphics->addUIForceInteraction(object_name, true);
	Eigen::VectorXd ui_interaction_torques_object;

	unsigned long long counter = 0;

	cout << endl
		 << "It is possible to interact with the displayed robot using the "
			"right mouse button. A line will be displayed, illustrating the "
			"applied force (or torque) on the robot, and the corresponding "
			"torque on the robot joint will be printed to terminal periodically"
		 << endl;
	cout << "Right mouse button: apply a force, scroll or press A or Z to "
			"push/pull the object"
		 << endl;
	cout << "Right mouse button + shift: apply a moment, scroll or press A or "
			"Z to to apply a moment in and out of the plane"
		 << endl;
	cout << endl;

	// while window is open:
	while (graphics->isWindowOpen())
	{
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
		ui_interaction_torques_robot = graphics->getUITorques(robot_name);
		ui_interaction_torques_object = graphics->getUITorques(object_name);

		if (counter % 50 == 0)
		{
			std::cout << "robot interaction torques: "
					  << ui_interaction_torques_robot.transpose() << std::endl;
			std::cout << "object interaction torques: "
					  << ui_interaction_torques_object.transpose() << std::endl;
			std::cout << std::endl;
		}

		counter++;
	}

	return 0;
}
