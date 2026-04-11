/**
 * \file SaiGraphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "SaiGraphics.h"

#include <deque>
#include <iostream>
#include <unordered_map>

#ifdef MACOSX
#include <filesystem>
#endif

#include "parser/UrdfToSaiGraphics.h"

using namespace std;
using namespace chai3d;

namespace
{
// custom aliases that capture the key function
#define ZOOM_IN_KEY GLFW_KEY_A
#define ZOOM_OUT_KEY GLFW_KEY_Z
#define CAMERA_RIGHT_KEY GLFW_KEY_RIGHT
#define CAMERA_LEFT_KEY GLFW_KEY_LEFT
#define CAMERA_UP_KEY GLFW_KEY_UP
#define CAMERA_DOWN_KEY GLFW_KEY_DOWN
#define NEXT_CAMERA_KEY GLFW_KEY_N
#define PREV_CAMERA_KEY GLFW_KEY_B
#define SHOW_CAMERA_POS_KEY GLFW_KEY_S
#define TOGGLE_FULLSCREEN_KEY GLFW_KEY_F
#define SWITCH_MAIN_SIDE_CAMERA_KEY GLFW_KEY_P
#define TRACKING_CAM_COMMAND_KEY GLFW_KEY_T

	// map to store key presses. The first bool is true if the key is pressed and
	// false otherwise, the second bool is used as a flag to know if the initial key
	// press has been consumed when something needs to happen only once when the key
	// is pressed and not continuously
	std::unordered_map<int, std::pair<bool, bool>> key_presses_map = {
		{ZOOM_IN_KEY, std::make_pair(false, true)},
		{ZOOM_OUT_KEY, std::make_pair(false, true)},
		{CAMERA_RIGHT_KEY, std::make_pair(false, true)},
		{CAMERA_LEFT_KEY, std::make_pair(false, true)},
		{CAMERA_UP_KEY, std::make_pair(false, true)},
		{CAMERA_DOWN_KEY, std::make_pair(false, true)},
		{NEXT_CAMERA_KEY, std::make_pair(false, true)},
		{PREV_CAMERA_KEY, std::make_pair(false, true)},
		{SHOW_CAMERA_POS_KEY, std::make_pair(false, true)},
		{TOGGLE_FULLSCREEN_KEY, std::make_pair(false, true)},
		{SWITCH_MAIN_SIDE_CAMERA_KEY, std::make_pair(false, true)},
		{TRACKING_CAM_COMMAND_KEY, std::make_pair(false, true)},
		{GLFW_KEY_LEFT_SHIFT, std::make_pair(false, true)},
		{GLFW_KEY_LEFT_ALT, std::make_pair(false, true)},
		{GLFW_KEY_LEFT_CONTROL, std::make_pair(false, true)},
		{GLFW_KEY_G, {false, true}},
		{GLFW_KEY_U, {false, true}},
		{GLFW_KEY_H, {false, true}},
		{GLFW_KEY_J, {false, true}},
		{GLFW_KEY_K, {false, true}},
		{GLFW_KEY_C, {false, true}},
		{GLFW_KEY_M, {false, true}},
	};

	std::unordered_map<int, std::pair<bool, bool>> mouse_button_presses_map = {
		{GLFW_MOUSE_BUTTON_LEFT, std::make_pair(false, true)},
		{GLFW_MOUSE_BUTTON_RIGHT, std::make_pair(false, true)},
		{GLFW_MOUSE_BUTTON_MIDDLE, std::make_pair(false, true)},
	};

	std::deque<double> mouse_scroll_buffer;

	bool is_pressed(int key)
	{
		if (key_presses_map.count(key) > 0)
		{
			return key_presses_map.at(key).first;
		}
		if (mouse_button_presses_map.count(key) > 0)
		{
			return mouse_button_presses_map.at(key).first;
		}
		return false;
	}

	bool consume_first_press(int key)
	{
		if (key_presses_map.count(key) > 0)
		{
			if (key_presses_map.at(key).first && key_presses_map.at(key).second)
			{
				key_presses_map.at(key).second = false;
				return true;
			}
		}
		if (mouse_button_presses_map.count(key) > 0)
		{
			if (mouse_button_presses_map.at(key).first &&
				mouse_button_presses_map.at(key).second)
			{
				mouse_button_presses_map.at(key).second = false;
				return true;
			}
		}
		return false;
	}

	// callback to print glfw errors
	void glfwError(int error, const char *description)
	{
		cerr << "GLFW Error: " << description << endl;
		exit(1);
	}

	// callback when a key is pressed
	void keySelect(GLFWwindow *window, int key, int scancode, int action,
				   int mods)
	{
		bool set = (action != GLFW_RELEASE);
		if (key == GLFW_KEY_ESCAPE)
		{
			// handle esc separately to exit application
			glfwSetWindowShouldClose(window, GL_TRUE);
		}
		else
		{
			if (key_presses_map.count(key) > 0)
			{
				key_presses_map.at(key).first = set;
				if (!set)
				{
					key_presses_map.at(key).second = true;
				}
			}
		}
	}

	// void keySelect(GLFWwindow *window, int key, int scancode, int action, int mods)
	// {
	// 	if (key == GLFW_KEY_ESCAPE)
	// 	{
	// 		glfwSetWindowShouldClose(window, GL_TRUE);
	// 		return;
	// 	}

	// 	// Track only known keys
	// 	if (key_presses_map.count(key) == 0)
	// 		return;

	// 	if (action == GLFW_PRESS)
	// 	{
	// 		key_presses_map[key].first = true;	 // key is now pressed
	// 		key_presses_map[key].second = false; // not yet consumed
	// 	}
	// 	else if (action == GLFW_RELEASE)
	// 	{
	// 		key_presses_map[key].first = false; // released
	// 											// don't reset second here — we reset it after consuming
	// 	}
	// }

	// callback when a mouse button is pressed
	void mouseClick(GLFWwindow *window, int button, int action, int mods)
	{
		bool set = (action != GLFW_RELEASE);
		if (mouse_button_presses_map.count(button) > 0)
		{
			mouse_button_presses_map.at(button).first = set;
			if (!set)
			{
				mouse_button_presses_map.at(button).second = true;
			}
		}
	}

	// callback when the mouse wheel is scrolled
	void mouseScroll(GLFWwindow *window, double xoffset, double yoffset)
	{
		if (yoffset != 0)
		{
			mouse_scroll_buffer.push_back(yoffset);
		}
		else if (xoffset != 0)
		{
			mouse_scroll_buffer.push_back(xoffset);
		}
	}

	bool glewInitialize()
	{
		bool ret = false;
#ifdef GLEW_VERSION
		// Initialize glew library
		if (glewInit() != GLEW_OK)
		{
			cout << "Failed to initialize glew library" << endl;
			cout << glewGetErrorString(ret) << endl;
			glfwTerminate();
		}
		else
		{
			ret = true;
		}
#endif
		return ret;
	}

	GLFWwindow *glfwInitialize(const std::string &window_name)
	{
		/*------- Set up visualization -------*/
		// set up error callback
		glfwSetErrorCallback(glfwError);

		// initialize GLFW
		if (!glfwInit())
			throw std::runtime_error("glfw init error");

		// retrieve resolution of computer display and position window accordingly
		GLFWmonitor *primary = glfwGetPrimaryMonitor();
		const GLFWvidmode *mode = glfwGetVideoMode(primary);

		// information about computer screen and GLUT display window
		int screenW = mode->width;
		int screenH = mode->height;
		int windowW = 0.5 * screenW;
		int windowH = 0.5 * screenH;
		int windowPosY = (screenH - windowH) / 2;
		int windowPosX = (screenW - windowW) / 2;

		// create window and make it current context
		glfwWindowHint(GLFW_VISIBLE, 0);

		GLFWwindow *window =
			glfwCreateWindow(windowW, windowH, window_name.c_str(), NULL, NULL);
		glfwSetWindowPos(window, windowPosX, windowPosY);
		glfwShowWindow(window);
		glfwMakeContextCurrent(window);
		glfwSwapInterval(0);

		return window;
	}
} // namespace

namespace SaiGraphics
{

	SaiGraphics::SaiGraphics(const std::string &path_to_world_file,
							 const std::string &window_name, bool verbose)
	{
		// initialize a chai world
		initializeWorld(path_to_world_file, verbose);
#ifdef MACOSX
		auto path = std::__fs::filesystem::current_path();
		initializeWindow(window_name);
		std::__fs::filesystem::current_path(path);
#else
		initializeWindow(window_name);
#endif
	}

	// dtor
	SaiGraphics::~SaiGraphics()
	{
		glfwDestroyWindow(_window);
		glfwTerminate();
		clearWorld();
		_world = NULL;
	}

	void SaiGraphics::resetWorld(const std::string &path_to_world_file,
								 const bool verbose)
	{
		clearWorld();
		initializeWorld(path_to_world_file, verbose);
	}

	void SaiGraphics::initializeWorld(const std::string &path_to_world_file,
									  const bool verbose)
	{
		_world = new chai3d::cWorld();
		Parser::UrdfToSaiGraphicsWorld(
			path_to_world_file, _world, _robot_filenames, _dyn_objects_pose,
			_static_objects_pose, _camera_frame_buffers, verbose);
		_current_camera_index = 0;
		for (auto it : _camera_frame_buffers)
		{
			_camera_names.push_back(it.first);
		}

		for (auto robot_filename : _robot_filenames)
		{
			// get robot base object in chai world
			cRobotBase *base = NULL;
			for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
			{
				if (robot_filename.first == _world->getChild(i)->m_name)
				{
					// cast to cRobotBase
					base = dynamic_cast<cRobotBase *>(_world->getChild(i));
					if (base != NULL)
					{
						break;
					}
				}
			}
			Eigen::Affine3d T_robot_base;
			T_robot_base.translation() = base->getLocalPos().eigen();
			T_robot_base.linear() = base->getLocalRot().eigen();
			_robot_models[robot_filename.first] =
				std::make_shared<SaiModel::SaiModel>(robot_filename.second);
			_robot_models[robot_filename.first]->setTRobotBase(T_robot_base);
			updateRobotGraphics(robot_filename.first,
								_robot_models[robot_filename.first]->q());
		}
		for (auto object_pose : _dyn_objects_pose)
		{
			_object_velocities[object_pose.first] =
				std::make_shared<Eigen::Vector6d>(Eigen::Vector6d::Zero());
		}
		_right_click_interaction_occurring = false;
	}

	void SaiGraphics::clearWorld()
	{
		delete _world;
		_robot_filenames.clear();
		_robot_models.clear();
		_dyn_objects_pose.clear();
		_static_objects_pose.clear();
		_object_velocities.clear();
		_camera_names.clear();
		_camera_frame_buffers.clear();
		_force_sensor_displays.clear();
		_ui_force_widgets.clear();
		_camera_link_attachments.clear();
		_light_link_attachments.clear();
	}

	void SaiGraphics::initializeWindow(const std::string &window_name)
	{

		_window = glfwInitialize(window_name);

		glewInitialize();

		// set callbacks
		glfwSetKeyCallback(_window, keySelect);
		glfwSetMouseButtonCallback(_window, mouseClick);
		glfwSetScrollCallback(_window, mouseScroll);
	}

	void SaiGraphics::setCameraPose(const std::string &camera_name,
									const Eigen::Affine3d &camera_pose)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: Camera [" << camera_name
				 << "] does not exists in the graphics world. Cannot set pose"
				 << endl;
			return;
		}
		// if (_camera_link_attachments.find(camera_name) !=
		// 	_camera_link_attachments.end())
		// {
		// 	cout << "WARNING: Cannot set pose for camera [" << camera_name
		// 		 << "] attached to a robot or object" << endl;
		// 	return;
		// }
		Vector3d pos = camera_pose.translation();
		Vector3d up = -camera_pose.rotation().col(1);
		Vector3d lookat = pos + camera_pose.linear().col(2);
		setCameraPoseInternal(camera_name, pos, up, lookat);
	}

	Eigen::Affine3d SaiGraphics::getCameraPose(const std::string &camera_name)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: Camera [" << camera_name
				 << "] does not exists in the graphics world. Cannot get pose"
				 << endl;
			return Affine3d::Identity();
		}
		Vector3d pos, up, lookat;
		getCameraPoseInternal(camera_name, pos, up, lookat);

		Matrix3d rotation = Matrix3d::Identity();
		rotation.col(1) = -up;
		rotation.col(2) = (lookat - pos).normalized();
		rotation.col(0) = rotation.col(1).cross(rotation.col(2));

		Affine3d camera_pose = Affine3d(rotation);
		camera_pose.translation() = pos;

		return camera_pose;
	}

	void SaiGraphics::attachCameraToRobotLink(
		const std::string &camera_name, const std::string &robot_name,
		const std::string &link_name, const Eigen::Affine3d &pose_in_link)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: camera [" << camera_name
				 << "] not found in graphics world, cannot attach to robot link"
				 << endl;
			return;
		}
		if (!robotExistsInWorld(robot_name, link_name))
		{
			cout << "WARNING: robot [" << robot_name << "] link [" << link_name
				 << "] not found in graphics world, cannot attach a camera to it"
				 << endl;
			return;
		}
		if (_camera_link_attachments.find(camera_name) !=
			_camera_link_attachments.end())
		{
			cout << "camera [" << camera_name
				 << "] already attached to a robot or object. detach first" << endl;
			return;
		}
		_camera_link_attachments[camera_name] =
			std::make_shared<CameraLinkAttachment>(robot_name, link_name,
												   pose_in_link);
	}

	void SaiGraphics::attachCameraToObject(const std::string &camera_name,
										   const std::string &object_name,
										   const Eigen::Affine3d &pose_in_object)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: camera [" << camera_name
				 << "] not found in graphics world, cannot attach to object"
				 << endl;
			return;
		}
		if (!dynamicObjectExistsInWorld(object_name) &&
			!staticObjectExistsInWorld(object_name))
		{
			cout << "WARNING: object [" << object_name
				 << "] object not found in graphics world, cannot attach a camera "
					"to it"
				 << endl;
			return;
		}
		if (_camera_link_attachments.find(camera_name) !=
			_camera_link_attachments.end())
		{
			cout << "camera [" << camera_name
				 << "] already attached to a robot or object. detach first" << endl;
			return;
		}
		_camera_link_attachments[camera_name] =
			std::make_shared<CameraLinkAttachment>(object_name, "", pose_in_object);
	}

	void SaiGraphics::detachCameraFromRobotOrObject(
		const std::string &camera_name)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: camera [" << camera_name
				 << "] not found in graphics world, cannot detach from robot or "
					"object"
				 << endl;
			return;
		}
		if (_camera_link_attachments.find(camera_name) ==
			_camera_link_attachments.end())
		{
			cout << "camera [" << camera_name
				 << "] not attached to any object or robot" << endl;
			return;
		}
		_camera_link_attachments.erase(camera_name);
	}

	void SaiGraphics::attachSpotLightToRobotLink(
		const std::string &light_name, const std::string &robot_name,
		const std::string &link_name, const Eigen::Affine3d &pose_in_link)
	{

		chai3d::cGenericLight *light = getLight(light_name);
		if (light == nullptr)
		{
			cout << "WARNING: light [" << light_name
				 << "] not found in graphics world, cannot attach to robot link"
				 << endl;
			return;
		}
		if (!robotExistsInWorld(robot_name, link_name))
		{
			cout << "WARNING: robot [" << robot_name << "] link [" << link_name
				 << "] not found in graphics world, cannot attach a light to it"
				 << endl;
			return;
		}
		if (_light_link_attachments.find(light_name) !=
			_light_link_attachments.end())
		{
			cout << "light [" << light_name
				 << "] already attached to a robot or object. detach first" << endl;
			return;
		}
		_light_link_attachments[light_name] =
			std::make_shared<LightLinkAttachment>(robot_name, link_name, pose_in_link);
	}

	void SaiGraphics::addForceSensorDisplay(
		const SaiModel::ForceSensorData &sensor_data)
	{
		if (findForceSensorDisplay(sensor_data.robot_or_object_name,
								   sensor_data.link_name) != -1)
		{
			std::cout << "\n\nWARNING: only one force sensor is supported per "
						 "link in SaiGraphics::addForceSensorDisplay. Not "
						 "adding the second one\n"
					  << std::endl;
			return;
		}
		if (robotExistsInWorld(sensor_data.robot_or_object_name,
							   sensor_data.link_name))
		{
			_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
				sensor_data.robot_or_object_name, sensor_data.link_name,
				sensor_data.transform_in_link,
				_robot_models.at(sensor_data.robot_or_object_name), _world));
		}
		else if (dynamicObjectExistsInWorld(sensor_data.robot_or_object_name))
		{
			_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
				sensor_data.robot_or_object_name, sensor_data.link_name,
				sensor_data.transform_in_link,
				_dyn_objects_pose.at(sensor_data.robot_or_object_name), _world));
		}
		else if (staticObjectExistsInWorld(sensor_data.robot_or_object_name))
		{
			_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
				sensor_data.robot_or_object_name, sensor_data.link_name,
				sensor_data.transform_in_link,
				_static_objects_pose.at(sensor_data.robot_or_object_name), _world));
		}
		else
		{
			std::cout << "\n\nWARNING: trying to add a force sensor display to an "
						 "unexisting robot or link in "
						 "SaiGraphics::addForceSensorDisplay\n"
					  << std::endl;
		}
	}

	void SaiGraphics::updateDisplayedForceSensor(
		const SaiModel::ForceSensorData &force_data)
	{
		int sensor_index = findForceSensorDisplay(force_data.robot_or_object_name,
												  force_data.link_name);
		if (sensor_index == -1)
		{
			throw std::invalid_argument(
				"no force sensor on robot " + force_data.robot_or_object_name +
				" on link " + force_data.link_name +
				". Impossible to update the displayed force in graphics world");
			return;
		}
		if (!_force_sensor_displays.at(sensor_index)
				 ->T_link_sensor()
				 .isApprox(force_data.transform_in_link))
		{
			throw std::invalid_argument(
				"transformation matrix between link and sensor inconsistent "
				"between the input force_data and the sensor_display in "
				"SaiGraphics::updateDisplayedForceSensor");
			return;
		}
		_force_sensor_displays.at(sensor_index)
			->update(force_data.force_world_frame, force_data.moment_world_frame);
	}

	bool SaiGraphics::robotExistsInWorld(const std::string &robot_name,
										 const std::string &link_name) const
	{
		auto it = _robot_models.find(robot_name);
		if (it == _robot_models.end())
		{
			return false;
		}
		if (link_name != "")
		{
			return _robot_models.at(robot_name)->isLinkInRobot(link_name);
		}
		return true;
	}

	bool SaiGraphics::dynamicObjectExistsInWorld(
		const std::string &object_name) const
	{
		auto it = _dyn_objects_pose.find(object_name);
		if (it == _dyn_objects_pose.end())
		{
			return false;
		}
		return true;
	}

	bool SaiGraphics::staticObjectExistsInWorld(
		const std::string &object_name) const
	{
		auto it = _static_objects_pose.find(object_name);
		if (it == _static_objects_pose.end())
		{
			return false;
		}
		return true;
	}

	void SaiGraphics::setScaleStaticObject(
		const std::string &object_name, const double scale)
	{
		if (!staticObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"static object " + object_name +
				" does not exist in graphics world. Cannot set scale.");
		}
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
		{
			if (_world->getChild(i)->m_name == object_name)
			{
				_world->getChild(i)->scale(scale);
				return;
			}
		}
	}

	bool SaiGraphics::cameraExistsInWorld(const std::string &camera_name) const
	{
		for (const std::string name : _camera_names)
		{
			if (name == camera_name)
			{
				return true;
			}
		}
		return false;
	}

	int SaiGraphics::findForceSensorDisplay(
		const std::string &robot_or_object_name,
		const std::string &link_name) const
	{
		for (int i = 0; i < _force_sensor_displays.size(); ++i)
		{
			if (_force_sensor_displays.at(i)->robot_or_object_name() ==
					robot_or_object_name &&
				_force_sensor_displays.at(i)->link_name() == link_name)
			{
				return i;
			}
		}
		return -1;
	}

	void SaiGraphics::addUIForceInteraction(
		const std::string &robot_or_object_name,
		const bool interact_at_object_center)
	{
		bool is_robot = robotExistsInWorld(robot_or_object_name);
		bool is_object = dynamicObjectExistsInWorld(robot_or_object_name);
		if (!is_robot && !is_object)
		{
			throw std::invalid_argument(
				"robot or dynamic object not found in "
				"SaiGraphics::addUIForceInteraction");
		}
		for (auto widget : _ui_force_widgets)
		{
			if (robot_or_object_name == widget->getRobotOrObjectName())
			{
				return;
			}
		}
		chai3d::cShapeLine *display_line = new chai3d::cShapeLine();
		_world->addChild(display_line);
		if (is_robot)
		{
			_ui_force_widgets.push_back(std::make_shared<UIForceWidget>(
				robot_or_object_name, interact_at_object_center,
				_robot_models[robot_or_object_name], display_line));
		}
		else
		{
			_ui_force_widgets.push_back(std::make_shared<UIForceWidget>(
				robot_or_object_name, interact_at_object_center,
				_dyn_objects_pose[robot_or_object_name],
				_object_velocities[robot_or_object_name], display_line));
		}
	}

	Eigen::VectorXd SaiGraphics::getUITorques(
		const std::string &robot_or_object_name)
	{
		bool is_robot = robotExistsInWorld(robot_or_object_name);
		bool is_object = dynamicObjectExistsInWorld(robot_or_object_name);
		if (!is_robot && !is_object)
		{
			throw std::invalid_argument(
				"robot or dynamic object not found in SaiGraphics::getUITorques");
		}
		for (auto widget : _ui_force_widgets)
		{
			if (robot_or_object_name == widget->getRobotOrObjectName())
			{
				return widget->getUIJointTorques();
			}
		}
		return is_robot ? Eigen::VectorXd::Zero(
							  _robot_models[robot_or_object_name]->dof())
						: Eigen::VectorXd::Zero(6);
	}

	Eigen::Vector6d SaiGraphics::getUIForceMoment(const std::string &robot_or_object_name)
	{
		bool is_robot = robotExistsInWorld(robot_or_object_name);
		bool is_object = dynamicObjectExistsInWorld(robot_or_object_name);
		if (!is_robot && !is_object)
		{
			throw std::invalid_argument(
				"robot or dynamic object not found in "
				"SaiGraphics::getUIForceMoment");
		}
		for (auto widget : _ui_force_widgets)
		{
			if (robot_or_object_name == widget->getRobotOrObjectName())
			{
				return widget->getAppliedForceMoment();
			}
		}
		return Eigen::Vector6d::Zero();
	}

	const std::vector<std::string> SaiGraphics::getRobotNames() const
	{
		std::vector<std::string> robot_names;
		for (const auto &it : _robot_filenames)
		{
			robot_names.push_back(it.first);
		}
		return robot_names;
	}

	const std::vector<std::string> SaiGraphics::getObjectNames() const
	{
		std::vector<std::string> object_names;
		for (const auto &it : _dyn_objects_pose)
		{
			object_names.push_back(it.first);
		}
		return object_names;
	}

	void SaiGraphics::renderBlackScreen()
	{
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glfwSwapBuffers(_window);
		glfwPollEvents();
	}

	cImagePtr SaiGraphics::getCameraImage(const std::string &camera_name,
										  const int width, const int height)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			cout << "WARNING: Camera [" << camera_name
				 << "] does not exists in the graphics world. Cannot get image"
				 << endl;
			return cImage::create();
		}

		_world->updateShadowMaps();
		_camera_frame_buffers.at(camera_name)->setSize(width, height);
		_camera_frame_buffers.at(camera_name)->renderView();
		cImagePtr image = cImage::create();
		_camera_frame_buffers.at(camera_name)->copyImageBuffer(image);
		return image;
	}

	void SaiGraphics::setCameraName(const std::string &camera_name)
	{
		for (int i = 0; i < _camera_names.size(); ++i)
		{
			if (_camera_names[i] == camera_name)
			{
				_current_camera_index = i;
				return;
			}
		}
		std::cout << "WARNING: Camera [" << camera_name << "] not found in graphics world." << std::endl;
	}

	void SaiGraphics::renderGraphicsWorld()
	{
		// toggles fullscreen if needed
		if (consume_first_press(TOGGLE_FULLSCREEN_KEY))
		{
			_fullscreen = !_fullscreen;

			GLFWmonitor *monitor = glfwGetPrimaryMonitor();
			const GLFWvidmode *mode = glfwGetVideoMode(monitor);

			if (_fullscreen)
			{
				// Get current window pos/size to restore later if needed
				// or use hardcoded 50% logic from glfwInitialize
				glfwSetWindowMonitor(_window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			}
			else
			{
				// Restore to a centered window (using the logic from your glfwInitialize function)
				int windowW = 0.5 * mode->width;
				int windowH = 0.5 * mode->height;
				int windowPosX = (mode->width - windowW) / 2;
				int windowPosY = (mode->height - windowH) / 2;
				glfwSetWindowMonitor(_window, NULL, windowPosX, windowPosY, windowW, windowH, 0);
			}
			glfwSwapInterval(0);
		}

		// swap camera if needed
		if (consume_first_press(NEXT_CAMERA_KEY))
		{
			cout << "Switching to camera " << _camera_names[(_current_camera_index + 1) % _camera_names.size()] << endl;
			_current_camera_index =
				(_current_camera_index + 1) % _camera_names.size();
		}
		if (consume_first_press(PREV_CAMERA_KEY))
		{
			cout << "Switching to camera " << _camera_names[(_current_camera_index - 1 + _camera_names.size()) % _camera_names.size()] << endl;
			_current_camera_index =
				(_current_camera_index - 1) % _camera_names.size();
		}

		if (consume_first_press(SWITCH_MAIN_SIDE_CAMERA_KEY))
		{
			string current_cam = _camera_names[_current_camera_index];
			string target_cam = "";

			auto it = _active_side_panels.find(current_cam);
			if (it != _active_side_panels.end() && !it->second.empty())
			{
				auto side_fb = it->second.front();
				for (auto const &[name, fb] : _camera_frame_buffers)
				{
					if (fb == side_fb)
					{
						target_cam = name;
						break;
					}
				}

				if (!target_cam.empty() && _active_side_panels.find(target_cam) == _active_side_panels.end())
				{
					createSidePanel(target_cam, current_cam);
				}
			}
			else
			{
				for (auto const &[main_name, panels] : _active_side_panels)
				{
					for (auto const &fb : panels)
					{
						if (_camera_frame_buffers[current_cam] == fb)
						{
							target_cam = main_name;
							break;
						}
					}
					if (!target_cam.empty())
						break;
				}
			}

			if (!target_cam.empty())
			{
				for (int i = 0; i < _camera_names.size(); ++i)
				{
					if (_camera_names[i] == target_cam)
					{
						_current_camera_index = i;
						break;
					}
				}
			}
		}

		if (consume_first_press(GLFW_KEY_C))
		{
			_show_collision_meshes = !_show_collision_meshes;

			// Calls the global toggle function we discussed earlier
			this->showCollisionMesh(_show_collision_meshes);

			std::cout << "Collision meshes: "
					  << (_show_collision_meshes ? "ON" : "OFF") << std::endl;
		}

		const std::string camera_name = _camera_names[_current_camera_index];

		// update graphics. this automatically waits for the correct amount of time
		glfwGetFramebufferSize(_window, &_window_width, &_window_height);
		glfwSwapBuffers(_window);
		// glFinish();

		// poll for events
		glfwPollEvents();

		// handle mouse button presses
		Eigen::Vector3d camera_pos, camera_lookat_point, camera_up_axis;
		getCameraPoseInternal(camera_name, camera_pos, camera_up_axis,
							  camera_lookat_point);
		Vector3d cam_depth_axis = camera_lookat_point - camera_pos;
		cam_depth_axis.normalize();
		Vector3d cam_right_axis = cam_depth_axis.cross(camera_up_axis);
		cam_right_axis.normalize();

		double cursorx, cursory;
		glfwGetCursorPos(_window, &cursorx, &cursory);
		double mouse_x_increment = (cursorx - _last_cursorx);
		double mouse_y_increment = (cursory - _last_cursory);

		// 0 - scroll event
		double scroll_value = 0.0;
		if (!mouse_scroll_buffer.empty())
		{
			scroll_value = mouse_scroll_buffer.front();
			mouse_scroll_buffer.pop_front();
		}

		// 1 - mouse right button to generate a force/torque
		if (is_pressed(GLFW_MOUSE_BUTTON_RIGHT))
		{
			if (consume_first_press(GLFW_MOUSE_BUTTON_RIGHT))
			{
				for (auto widget : _ui_force_widgets)
				{
					widget->setEnable(true);
				}
			}

			int wwidth_scr, wheight_scr;
			glfwGetWindowSize(_window, &wwidth_scr, &wheight_scr);

			int viewx = floor(cursorx / wwidth_scr * _window_width);
			int viewy = floor(cursory / wheight_scr * _window_height);

			for (auto widget : _ui_force_widgets)
			{
				if (widget->getState() == UIForceWidget::Active)
				{
					_right_click_interaction_occurring = true;
				}

				double depth_change = 0.0;
				if (_right_click_interaction_occurring)
				{
					depth_change = 0.1 * scroll_value;
					if (is_pressed(ZOOM_IN_KEY))
					{
						depth_change += 0.05;
					}
					if (is_pressed(ZOOM_OUT_KEY))
					{
						depth_change -= 0.05;
					}
				}
				if (is_pressed(GLFW_KEY_LEFT_SHIFT))
				{
					widget->setMomentMode();
				}
				else
				{
					widget->setForceMode();
				}
				widget->setInteractionParams(getCamera(camera_name), viewx,
											 _window_height - viewy, _window_width,
											 _window_height, depth_change);
			}
		}
		else
		{
			for (auto widget : _ui_force_widgets)
			{
				widget->setEnable(false);
			}
			_right_click_interaction_occurring = false;
		}

		// 2 - mouse left button for camera motion
		if (!_right_click_interaction_occurring)
		{
			if (is_pressed(GLFW_MOUSE_BUTTON_LEFT))
			{
				if (is_pressed(GLFW_KEY_LEFT_CONTROL))
				{
					Eigen::Vector3d cam_motion =
						0.01 * (mouse_x_increment * cam_right_axis -
								mouse_y_increment * camera_up_axis);
					camera_pos -= cam_motion;
					camera_lookat_point -= cam_motion;
				}
				else if (is_pressed(GLFW_KEY_LEFT_ALT) ||
						 is_pressed(GLFW_KEY_LEFT_SHIFT))
				{
					Eigen::Vector3d cam_motion =
						0.02 * mouse_y_increment * cam_depth_axis;
					camera_pos -= cam_motion;
					camera_lookat_point -= cam_motion;
				}
				else
				{
					Matrix3d m_tilt;
					m_tilt = AngleAxisd(0.006 * mouse_y_increment, -cam_right_axis);
					camera_pos = camera_lookat_point +
								 m_tilt * (camera_pos - camera_lookat_point);
					Matrix3d m_pan;
					m_pan = AngleAxisd(0.006 * mouse_x_increment, -camera_up_axis);
					camera_pos = camera_lookat_point +
								 m_pan * (camera_pos - camera_lookat_point);
					camera_up_axis = m_pan * camera_up_axis;
				}
			}
			if (is_pressed(GLFW_MOUSE_BUTTON_MIDDLE))
			{
				Eigen::Vector3d cam_motion =
					0.01 * (mouse_x_increment * cam_right_axis -
							mouse_y_increment * camera_up_axis);
				camera_pos -= cam_motion;
				camera_lookat_point -= cam_motion;
			}

			// 3 - mouse scrolling for zoom
			camera_pos += 0.2 * scroll_value * cam_depth_axis;
			camera_lookat_point += 0.2 * scroll_value * cam_depth_axis;

			// handle keyboard key presses
			if (is_pressed(CAMERA_RIGHT_KEY))
			{
				camera_pos += 0.05 * cam_right_axis;
				camera_lookat_point += 0.05 * cam_right_axis;
			}
			if (is_pressed(CAMERA_LEFT_KEY))
			{
				camera_pos -= 0.05 * cam_right_axis;
				camera_lookat_point -= 0.05 * cam_right_axis;
			}
			if (is_pressed(CAMERA_UP_KEY))
			{
				camera_pos += 0.05 * camera_up_axis;
				camera_lookat_point += 0.05 * camera_up_axis;
			}
			if (is_pressed(CAMERA_DOWN_KEY))
			{
				camera_pos -= 0.05 * camera_up_axis;
				camera_lookat_point -= 0.05 * camera_up_axis;
			}
			if (is_pressed(ZOOM_IN_KEY))
			{
				camera_pos += 0.1 * cam_depth_axis;
				camera_lookat_point += 0.1 * cam_depth_axis;
			}
			if (is_pressed(ZOOM_OUT_KEY))
			{
				camera_pos -= 0.1 * cam_depth_axis;
				camera_lookat_point -= 0.1 * cam_depth_axis;
			}

			if (consume_first_press(SHOW_CAMERA_POS_KEY))
			{
				cout << endl;
				cout << "<camera name=\"" << camera_name << "\">" << endl;
				cout << "	<position xyz=\"" << camera_pos.transpose() << "\" />"
					 << endl;
				cout << "	<lookat xyz=\"" << camera_lookat_point.transpose()
					 << "\" />" << endl;
				cout << "	<vertical xyz=\"" << camera_up_axis.transpose()
					 << "\" />" << endl;
				cout << "</camera>" << endl;
				cout << endl;
			}
		}

		setCameraPoseInternal(camera_name, camera_pos, camera_up_axis,
							  camera_lookat_point);
		glfwGetCursorPos(_window, &_last_cursorx, &_last_cursory);

		// if camera is attached to a robot link or object, override the pose
		if (_camera_link_attachments.find(camera_name) !=
			_camera_link_attachments.end())
		{
			const auto attachment = _camera_link_attachments.at(camera_name);

			Affine3d camera_pose;
			if (attachment->link_name == "")
			{
				camera_pose = getObjectPose(attachment->model_name) *
							  attachment->pose_in_link;
			}
			else
			{
				camera_pose =
					_robot_models.at(attachment->model_name)
						->transformInWorld(attachment->link_name,
										   attachment->pose_in_link.translation(),
										   attachment->pose_in_link.rotation());
			}
			setCameraPose(camera_name, camera_pose);
		}

		// update shadow maps
		_world->updateShadowMaps();

		for (const auto &item : _camera_link_attachments)
		{
			string cam_name = item.first;
			auto attachment = item.second;
			Affine3d camera_pose;

			if (attachment->link_name.empty())
			{
				camera_pose = getObjectPose(attachment->model_name) * attachment->pose_in_link;
			}
			else
			{
				if (_robot_models.find(attachment->model_name) != _robot_models.end())
				{
					camera_pose = _robot_models.at(attachment->model_name)->transformInWorld(attachment->link_name, attachment->pose_in_link.translation(), attachment->pose_in_link.rotation());
				}
			}
			setCameraPose(cam_name, camera_pose);
		}

		// update the lights position if attached to robot link
		for (const auto &item : _light_link_attachments)
		{
			string light_name = item.first;
			auto attachment = item.second;
			Affine3d light_pose;

			if (_robot_models.find(attachment->model_name) != _robot_models.end())
			{
				light_pose = _robot_models.at(attachment->model_name)->transformInWorld(attachment->link_name, attachment->pose_in_link.translation(), attachment->pose_in_link.rotation());
				auto light = getLight(light_name);
				light->setLocalPos(cVector3d(light_pose.translation()));
				light->setLocalRot(cMatrix3d(light_pose.rotation()));
			}
		}

		auto it = _active_side_panels.find(camera_name);
		if (it != _active_side_panels.end())
		{
			const double SIZE_RATIO = 0.30;
			const int margin = 30;
			const int borderWidth = 3;

			int pWidth = static_cast<int>(_window_width * SIZE_RATIO);
			int pHeight = static_cast<int>(_window_height * SIZE_RATIO);

			for (auto &fb : it->second)
			{
				fb->setSize(pWidth - (2 * borderWidth), pHeight - (2 * borderWidth));
				fb->renderView();

				auto main_cam = getCamera(camera_name);
				for (int i = 0; i < main_cam->m_frontLayer->getNumChildren(); ++i)
				{
					auto borderPanel = dynamic_cast<chai3d::cPanel *>(main_cam->m_frontLayer->getChild(i));
					if (borderPanel && borderPanel->getNumChildren() > 0)
					{
						auto viewPanel = dynamic_cast<chai3d::cViewPanel *>(borderPanel->getChild(0));
						if (viewPanel && viewPanel->getFrameBuffer() == fb)
						{
							borderPanel->setSize(pWidth, pHeight);
							borderPanel->setLocalPos(_window_width - pWidth - margin, _window_height - pHeight - margin);

							viewPanel->setSize(pWidth - (2 * borderWidth), pHeight - (2 * borderWidth));
							viewPanel->setLocalPos(borderWidth, borderWidth);
						}
					}
				}
			}
		}

		render(camera_name);
	}

	static void updateGraphicsLink(
		cRobotLink *link, std::shared_ptr<SaiModel::SaiModel> robot_model)
	{
		cVector3d local_pos;
		cMatrix3d local_rot;

		// get link name
		std::string link_name = link->m_name;

		// get chai parent name
		cGenericObject *parent = link->getParent();
		std::string parent_name = parent->m_name;

		// if parent is cRobotBase, then simply get transform relative to base from
		// model
		if (dynamic_cast<cRobotBase *>(parent) != NULL)
		{
			Eigen::Affine3d T = robot_model->transform(link_name);
			local_pos = cVector3d(T.translation());
			local_rot = cMatrix3d(T.rotation());
		}
		else if (dynamic_cast<cRobotLink *>(parent) != NULL)
		{
			// if parent is cRobotLink, then calculate transform for both links,
			// then apply inverse transform
			Eigen::Affine3d T_me, T_parent, T_rel;
			T_me = robot_model->transform(link_name);
			T_parent = robot_model->transform(parent_name);
			T_rel = T_parent.inverse() * T_me;
			local_pos = cVector3d(T_rel.translation());
			local_rot = cMatrix3d(T_rel.rotation());
		}
		else
		{
			cerr << "Parent to link " << link_name << " is neither link nor base"
				 << endl;
			abort();
			// TODO: throw exception
		}
		link->setLocalPos(local_pos);
		link->setLocalRot(local_rot);

		// call on children
		cRobotLink *child;
		for (unsigned int i = 0; i < link->getNumChildren(); ++i)
		{
			child = dynamic_cast<cRobotLink *>(link->getChild(i));
			if (child != NULL)
			{
				updateGraphicsLink(child, robot_model);
			}
		}
	}

	// update frame for a particular robot
	void SaiGraphics::updateRobotGraphics(const std::string &robot_name,
										  const Eigen::VectorXd &joint_angles)
	{
		auto it = _robot_models.find(robot_name);
		if (it == _robot_models.end())
		{
			throw std::invalid_argument(
				"Robot not found in SaiGraphics::updateRobotGraphics");
		}
		updateRobotGraphics(
			robot_name, joint_angles,
			Eigen::VectorXd::Zero(_robot_models.at(robot_name)->dof()));
	}

	void SaiGraphics::updateRobotGraphics(
		const std::string &robot_name, const Eigen::VectorXd &joint_angles,
		const Eigen::VectorXd &joint_velocities)
	{
		// update corresponfing robot model
		auto it = _robot_models.find(robot_name);
		if (it == _robot_models.end())
		{
			throw std::invalid_argument(
				"Robot not found in SaiGraphics::updateRobotGraphics");
		}
		auto robot_model = _robot_models.at(robot_name);
		if (joint_angles.size() != robot_model->qSize())
		{
			throw std::invalid_argument(
				"size of joint angles inconsistent with robot model in "
				"SaiGraphics::updateRobotGraphics");
		}
		if (joint_velocities.size() != robot_model->dof())
		{
			throw std::invalid_argument(
				"size of joint velocities inconsistent with robot model in "
				"SaiGraphics::updateRobotGraphics");
		}
		robot_model->setQ(joint_angles);
		robot_model->setDq(joint_velocities);
		robot_model->updateKinematics();

		// get robot base object in chai world
		cRobotBase *base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
		{
			if (robot_name == _world->getChild(i)->m_name)
			{
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase *>(_world->getChild(i));
				if (base != NULL)
				{
					break;
				}
			}
		}
		if (base == NULL)
		{
			// TODO: throw exception
			cerr << "Could not find robot in chai world: " << robot_name << endl;
			abort();
		}
		// recursively update graphics for all children
		cRobotLink *link;
		for (unsigned int i = 0; i < base->getNumChildren(); ++i)
		{
			link = dynamic_cast<cRobotLink *>(base->getChild(i));
			if (link != NULL)
			{
				updateGraphicsLink(link, robot_model);
			}
		}
	}

	void SaiGraphics::updateObjectGraphics(
		const std::string &object_name, const Eigen::Affine3d &object_pose,
		const Eigen::Vector6d &object_velocity)
	{
		if (!dynamicObjectExistsInWorld(object_name) && !staticObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"object not found in SaiGraphics::updateObjectGraphics");
		}
		cGenericObject *object = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
		{
			if (object_name == _world->getChild(i)->m_name)
			{
				// cast to cRobotBase
				object = _world->getChild(i);
				if (object != NULL)
				{
					break;
				}
			}
		}
		if (object == NULL)
		{
			// TODO: throw exception
			cerr << "Could not find object in chai world: " << object_name << endl;
			abort();
		}

		// update pose
		if (dynamicObjectExistsInWorld(object_name))
		{
			*_dyn_objects_pose.at(object_name) = object_pose;
			*_object_velocities.at(object_name) = object_velocity;
		}
		else if (staticObjectExistsInWorld(object_name))
		{
			*_static_objects_pose.at(object_name) = object_pose;
		}
		object->setLocalPos(object_pose.translation());
		object->setLocalRot(object_pose.rotation());
	}

	Eigen::VectorXd SaiGraphics::getRobotJointPos(const std::string &robot_name)
	{
		auto it = _robot_models.find(robot_name);
		if (it == _robot_models.end())
		{
			throw std::invalid_argument(
				"robot not found in SaiGraphics::getRobotJointPos");
		}
		return _robot_models[robot_name]->q();
	}

	Eigen::Affine3d SaiGraphics::getObjectPose(const std::string &object_name)
	{
		if (!dynamicObjectExistsInWorld(object_name))
		{
			throw std::invalid_argument(
				"dynamic object not found in SaiGraphics::getObjectPose");
		}
		return *_dyn_objects_pose.at(object_name);
	}

	void SaiGraphics::render(const std::string &camera_name)
	{
		auto camera = getCamera(camera_name);
		// TODO: support link mounted cameras
		// TODO: support stereo. see cCamera::renderView
		//	to do so, we need to search through the descendent tree
		// render view from this camera
		// NOTE: we don't use the display context id right now since chai no longer
		// supports it in 3.2.0
		camera->renderView(_window_width, _window_height);
	}

	// get current camera pose
	void SaiGraphics::getCameraPoseInternal(const std::string &camera_name,
											Eigen::Vector3d &ret_position,
											Eigen::Vector3d &ret_vertical_axis,
											Eigen::Vector3d &ret_lookat_point)
	{
		auto camera = getCamera(camera_name);
		cVector3d pos, vert, lookat;
		pos = camera->getLocalPos();
		ret_position << pos.x(), pos.y(), pos.z();
		vert = camera->getUpVector();
		ret_vertical_axis << vert.x(), vert.y(), vert.z();
		lookat = camera->getLookVector();
		ret_lookat_point << lookat.x(), lookat.y(), lookat.z();
		ret_lookat_point += ret_position;
	}

	// set camera pose
	void SaiGraphics::setCameraPoseInternal(const std::string &camera_name,
											const Eigen::Vector3d &position,
											const Eigen::Vector3d &vertical_axis,
											const Eigen::Vector3d &lookat_point)
	{
		auto camera = getCamera(camera_name);
		cVector3d pos(position[0], position[1], position[2]);
		cVector3d vert(vertical_axis[0], vertical_axis[1], vertical_axis[2]);
		cVector3d look(lookat_point[0], lookat_point[1], lookat_point[2]);
		camera->set(pos, look, vert);
	}

	// get camera object
	cCamera *SaiGraphics::getCamera(const std::string &camera_name)
	{
		if (!cameraExistsInWorld(camera_name))
		{
			throw std::invalid_argument(
				"camera not found in SaiGraphics::getCamera");
		}
		return _camera_frame_buffers.at(camera_name)->getCamera();
	}

	cRobotLink *SaiGraphics::findLinkObjectInParentLinkRecursive(
		cRobotLink *parent, const std::string &link_name)
	{
		// call on children
		cRobotLink *child;
		cRobotLink *ret_link = NULL;
		for (unsigned int i = 0; i < parent->getNumChildren(); ++i)
		{
			child = dynamic_cast<cRobotLink *>(parent->getChild(i));
			if (child != NULL)
			{
				if (child->m_name == link_name)
				{
					return child;
				}
				else
				{
					cRobotLink *found = findLinkObjectInParentLinkRecursive(child, link_name);
					if (found != NULL)
						return found;
				}
			}
		}
		return NULL;
	}

	cRobotLink *SaiGraphics::findLink(const std::string &robot_name,
									  const std::string &link_name)
	{
		// get robot base
		cRobotBase *base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
		{
			if (robot_name == _world->getChild(i)->m_name)
			{
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase *>(_world->getChild(i));
				if (base != NULL)
				{
					break;
				}
			}
		}
		if (base == NULL)
		{
			// TODO: throw exception
			cerr << "Could not find robot in chai world: " << robot_name << endl;
			abort();
		}

		cRobotLink *target_link = NULL;
		// get target link
		cRobotLink *base_link;
		for (unsigned int i = 0; i < base->getNumChildren(); ++i)
		{
			base_link = dynamic_cast<cRobotLink *>(base->getChild(i));
			if (base_link != NULL)
			{
				if (base_link->m_name == link_name)
				{
					target_link = base_link;
					break;
				}
				else
				{
					target_link =
						findLinkObjectInParentLinkRecursive(base_link, link_name);
					if (target_link != NULL)
					{
						break;
					}
				}
			}
		}
		return target_link;
	}

	void SaiGraphics::showLinkFrameRecursive(cRobotLink *parent, bool show_frame,
											 const double frame_pointer_length)
	{
		// call on children
		cRobotLink *child;
		for (unsigned int i = 0; i < parent->getNumChildren(); ++i)
		{
			child = dynamic_cast<cRobotLink *>(parent->getChild(i));
			if (child != NULL)
			{
				child->setFrameSize(frame_pointer_length, false);
				child->setShowFrame(show_frame, false);
				showLinkFrameRecursive(child, show_frame, frame_pointer_length);
			}
		}
	}

	void SaiGraphics::showLinkFrame(bool show_frame,
									const std::string &robot_or_object_name,
									const std::string &link_name,
									const double frame_pointer_length)
	{
		if (link_name.empty())
		{ // apply to all links
			cGenericObject *base = NULL;
			for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
			{
				if (robot_or_object_name == _world->getChild(i)->m_name)
				{
					base = _world->getChild(i);
					if (base != NULL)
					{
						break;
					}
				}
			}
			if (base == NULL)
			{
				cerr << "Could not find robot in chai world: "
					 << robot_or_object_name << ". Cannot show frame." << endl;
			}
			base->setFrameSize(frame_pointer_length, false);
			base->setShowFrame(show_frame, false);
			// get target link
			cRobotLink *base_link;
			for (unsigned int i = 0; i < base->getNumChildren(); ++i)
			{
				base_link = dynamic_cast<cRobotLink *>(base->getChild(i));
				if (base_link != NULL)
				{
					base_link->setFrameSize(frame_pointer_length, false);
					base_link->setShowFrame(show_frame, false);
					showLinkFrameRecursive(base_link, show_frame,
										   frame_pointer_length);
				}
			}
		}
		else
		{
			auto target_link = findLink(robot_or_object_name, link_name);
			if (target_link == NULL)
			{
				cerr << "Could not find link " << link_name
					 << " in robot " << robot_or_object_name
					 << ". Cannot show frame." << endl;
			}
			else
			{
				target_link->setFrameSize(frame_pointer_length, false);
				target_link->setShowFrame(show_frame, false);
			}
		}
	}

	void SaiGraphics::showWireMesh(bool show_wiremesh,
								   const std::string &robot_or_object_name,
								   const std::string &link_name)
	{
		if (link_name.empty())
		{ // apply to all links
			cGenericObject *base = NULL;
			for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
			{
				if (robot_or_object_name == _world->getChild(i)->m_name)
				{
					base = _world->getChild(i);
					if (base != NULL)
					{
						break;
					}
				}
			}
			if (base == NULL)
			{
				cerr << "Could not find robot or object in chai graphics world: "
					 << robot_or_object_name << ". Cannot show wire mesh." << endl;
			}
			base->setWireMode(show_wiremesh, true);
		}
		else
		{
			auto target_link = findLink(robot_or_object_name, link_name);
			target_link->setWireMode(show_wiremesh, true);
		}
	}

	void SaiGraphics::applyCollisionVisibilityRecursive(chai3d::cGenericObject *parent, bool show)
	{
		for (unsigned int i = 0; i < parent->getNumChildren(); ++i)
		{
			cGenericObject *child = parent->getChild(i);

			if (child->m_name == parent->m_name + "_collision" ||
				child->m_name.find("_collision") != std::string::npos)
			{
				child->setShowEnabled(show, true);
			}

			applyCollisionVisibilityRecursive(child, show);
		}
	}

	void SaiGraphics::showCollisionMesh(bool show_collisionmesh)
	{
		// _world is the root of the Chai3D scene graph.
		// This will crawl through every single robot, static object, and dynamic object
		// in the scene and toggle anything with "_collision" in its name.
		applyCollisionVisibilityRecursive(_world, show_collisionmesh);
	}

	void SaiGraphics::showCollisionMesh(bool show_collisionmesh,
										const std::string &robot_or_object_name,
										const std::string &link_name)
	{
		// Case: Apply to all links
		if (link_name.empty())
		{
			// Finds the robot base
			cGenericObject *base = NULL;
			for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
			{
				if (robot_or_object_name == _world->getChild(i)->m_name)
				{
					base = _world->getChild(i);
					break;
				}
			}

			if (base == NULL)
			{
				cerr << "Could not find robot or object in chai graphics world: "
					 << robot_or_object_name << ". Cannot show collision mesh."
					 << endl;
				return;
			}

			applyCollisionVisibilityRecursive(base, show_collisionmesh);
		}

		// Case: Apply to a specific link
		else
		{
			auto target_link = findLink(robot_or_object_name, link_name);
			if (target_link == NULL)
			{
				cerr << "Could not find link " << link_name
					 << " in robot " << robot_or_object_name
					 << ". Cannot show collision mesh." << endl;
				return;
			}

			// Find the collision mesh child of this specific link
			cGenericObject *child;
			for (unsigned int i = 0; i < target_link->getNumChildren(); ++i)
			{
				child = target_link->getChild(i);
				if (child->m_name == link_name + "_collision")
				{
					child->setShowEnabled(show_collisionmesh, false);
					break;
				}
			}
		}
	}

	void SaiGraphics::setRenderingEnabled(const bool rendering_enabled,
										  const string robot_or_object_name,
										  const string link_name)
	{
		if (link_name.empty())
		{ // apply to all links
			cGenericObject *base = NULL;
			for (unsigned int i = 0; i < _world->getNumChildren(); ++i)
			{
				if (robot_or_object_name == _world->getChild(i)->m_name)
				{
					base = _world->getChild(i);
					if (base != NULL)
					{
						break;
					}
				}
			}
			if (base == NULL)
			{
				cerr << "Could not find robot or object in chai graphics world: "
					 << robot_or_object_name << ". Cannot enable/disable rendering."
					 << endl;
			}
			base->setEnabled(rendering_enabled, true);
		}
		else
		{
			auto target_link = findLink(robot_or_object_name, link_name);
			target_link->setEnabled(rendering_enabled, false);
			cGenericObject *child;
			for (unsigned int i = 0; i < target_link->getNumChildren(); ++i)
			{
				child = target_link->getChild(i);
				// only apply to children that are visual elements (supposed to have
				// the same name), not children links (which will have different
				// names)
				if (child->m_name == link_name)
				{
					child->setEnabled(rendering_enabled, false);
				}
			}
		}
	}

	// this function takes as input a list of eigen:Vector3d points and creates a chai3d cMultiSegment
	chai3d::cMultiSegment *SaiGraphics::createMultiSegment(const std::vector<Eigen::Vector3d> &points, chai3d::cColorf color)
	{
		// create a new chai3d multi-segment
		auto *multi_segment = new chai3d::cMultiSegment();
		// add the multi-segment to the world
		_world->addChild(multi_segment);

		// add each point to the multi-segment
		for (int i = 0; i < points.size() - 1; i++)
		{
			// create vertex 0
			int index0 = multi_segment->newVertex(points[i](0), points[i](1), points[i](2));

			// create vertex 1
			int index1 = multi_segment->newVertex(points[i + 1](0), points[i + 1](1), points[i + 1](2));

			// create segment
			multi_segment->newSegment(index0, index1);
		}
		// position object
		multi_segment->setLocalPos(0.0, 0.0, 0.0);

		// set segment properties
		multi_segment->setLineColor(color);
		multi_segment->setLineWidth(4.0);

		// use display list to optimize graphic rendering performance
		multi_segment->setUseDisplayList(true);
		return multi_segment;
	}

	void SaiGraphics::updateMultiSegment(chai3d::cMultiSegment *multi_segment, const Eigen::Vector3d &point, chai3d::cColorf color, bool segment_creation)
	{
		int numVertices = multi_segment->m_vertices->getNumElements();

		if (numVertices == 0)
		{
			return;
		}

		chai3d::cVector3d last_vertex = multi_segment->m_vertices->getLocalPos(numVertices - 1);

		// add each point to the multi-segment
		// create vertex 0
		int index0 = multi_segment->newVertex(last_vertex(0), last_vertex(1), last_vertex(2));

		// create vertex 1
		int index1 = multi_segment->newVertex(point(0), point(1), point(2));

		if (segment_creation)
		{
			// create segment
			multi_segment->newSegment(index0, index1);
		}
		// set segment properties
		multi_segment->setLineColor(color);
		multi_segment->setLineWidth(4.0);

		// use display list to optimize graphic rendering performance
		multi_segment->setUseDisplayList(true);
	}

	chai3d::cMultiSegment *SaiGraphics::createLineSegment(const Eigen::Vector3d &point_start,
														  const Eigen::Vector3d &point_end,
														  chai3d::cColorf color, float line_width, bool setShowEnable)
	{
		// create a new chai3d multi-segment
		auto *multi_segment = new chai3d::cMultiSegment();

		// add the multi-segment to the world
		_world->addChild(multi_segment);

		// create vertex 0
		int index0 = multi_segment->newVertex(point_start(0), point_start(1), point_start(2));

		// create vertex 1
		int index1 = multi_segment->newVertex(point_end(0), point_end(1), point_end(2));

		// create segment
		multi_segment->newSegment(index0, index1);

		// position object
		multi_segment->setLocalPos(0.0, 0.0, 0.0);

		// set segment properties
		multi_segment->setLineColor(color);
		multi_segment->setLineWidth(line_width);

		// use display list to optimize graphic rendering performance
		multi_segment->setUseDisplayList(true);

		multi_segment->setShowEnabled(setShowEnable);

		return multi_segment;
	}

	// now create a function to update the linesegment
	void SaiGraphics::updateLineSegment(chai3d::cMultiSegment *multi_segment,
										const Eigen::Vector3d &point_start,
										const Eigen::Vector3d &point_end,
										chai3d::cColorf color, float line_width, bool setShowEnable)
	{
		multi_segment->setShowEnabled(setShowEnable);
		// clear existing vertices and segments
		multi_segment->clear();

		// create vertex 0
		int index0 = multi_segment->newVertex(point_start(0), point_start(1), point_start(2));

		// create vertex 1
		int index1 = multi_segment->newVertex(point_end(0), point_end(1), point_end(2));

		// create segment
		multi_segment->newSegment(index0, index1);

		// set segment properties
		multi_segment->setLineColor(color);
		multi_segment->setLineWidth(line_width);

		// use display list to optimize graphic rendering performance
		multi_segment->setUseDisplayList(true);
	}

	chai3d::cShapeSphere *SaiGraphics::createGoalSphere(const Eigen::Vector3d &position,
														const double radius,
														chai3d::cColorf color, bool setShowEnable)
	{
		auto *sphere = new chai3d::cShapeSphere(radius);
		_world->addChild(sphere);

		sphere->setLocalPos(position(0), position(1), position(2));

		sphere->m_material->setColor(color);
		// less shininess
		sphere->m_material->setShininess(100);

		sphere->setUseDisplayList(true);
		sphere->setShowEnabled(setShowEnable);
		return sphere;
	}

	void SaiGraphics::updateGoalSphere(chai3d::cShapeSphere *sphere, const Eigen::Vector3d &position, bool setShowEnable, bool setShowFrame, const Eigen::Matrix3d &orientation)
	{
		sphere->setShowEnabled(setShowEnable);
		sphere->setShowFrame(setShowFrame);
		sphere->setLocalPos(position(0), position(1), position(2));
		sphere->setLocalRot(cMatrix3d(orientation));
	}

	chai3d::cMesh *SaiGraphics::createEllipsoid(const Vector3d &scaleVector, const Vector3d &u, const Vector3d &v, const Vector3d &w, const Vector3d &position, cColorf &color)
	{
		// create a new mesh first:
		auto ellipsoid = new chai3d::cMesh();
		chai3d::cCreateEllipsoid(ellipsoid, scaleVector(0), scaleVector(1), scaleVector(2));
		ellipsoid->setUseTransparency(true);
		ellipsoid->m_material->setColor(color);
		ellipsoid->setLocalPos(position(0), position(1), position(2));
		// use u v w to create rotation matrix
		Matrix3d rot;
		rot.col(0) = u;
		rot.col(1) = v;
		rot.col(2) = w;
		ellipsoid->setLocalRot(cMatrix3d(rot));
		_world->addChild(ellipsoid);
		return ellipsoid;
	}

	void SaiGraphics::setBackgroundImage(const std::string &image_path, const std::string &camera_name)
	{
		auto applyToCamera = [&](const std::string &name)
		{
			chai3d::cCamera *camera = getCamera(name);
			if (camera)
			{
				chai3d::cBackground *background = new chai3d::cBackground();
				camera->m_backLayer->addChild(background);

				if (!background->loadFromFile(image_path))
				{
					std::cout << "Error - Image failed to load correctly for camera: " << name << std::endl;
				}
			}
			else
			{
				std::cerr << "Warning: Camera '" << name << "' not found." << std::endl;
			}
		};

		if (camera_name == "default_camera")
		{
			for (const std::string &name : _camera_names)
			{
				applyToCamera(name);
			}
		}
		else
		{
			applyToCamera(camera_name);
		}
	}

	void SaiGraphics::setFrontgroundImage(const std::string &image_path, const std::string &camera_name)
	{
		auto applyToCamera = [&](const std::string &name)
		{
			chai3d::cCamera *camera = getCamera(name);
			if (camera)
			{
				// 1. Create the background
				chai3d::cBackground *background = new chai3d::cBackground();
				camera->m_frontLayer->addChild(background);

				// 2. IMPORTANT: Enable transparency before or after loading
				background->setUseTransparency(true);

				if (!background->loadFromFile(image_path))
				{
					std::cout << "Error - Image failed to load correctly: " << name << std::endl;
				}
			}
			else
			{
				std::cerr << "Warning: Camera '" << name << "' not found." << std::endl;
			}
		};

		if (camera_name == "default_camera")
		{
			for (const std::string &name : _camera_names)
				applyToCamera(name);
		}
		else
		{
			applyToCamera(camera_name);
		}
	}

	void SaiGraphics::removeFrontgroundImage(const std::string &camera_name)
	{
		auto removeFromCamera = [&](const std::string &name)
		{
			chai3d::cCamera *camera = getCamera(name);
			if (!camera)
			{
				std::cerr << "Warning: Camera '" << name << "' not found." << std::endl;
				return;
			}

			auto frontLayer = camera->m_frontLayer;

			for (int i = frontLayer->getNumChildren() - 1; i >= 0; --i)
			{
				chai3d::cGenericObject *obj = frontLayer->getChild(i);

				if (dynamic_cast<chai3d::cBackground *>(obj))
				{
					frontLayer->removeChild(obj);
					delete obj;
				}
			}
		};

		if (camera_name == "default_camera")
		{
			for (const std::string &name : _camera_names)
			{
				removeFromCamera(name);
			}
		}
		else
		{
			removeFromCamera(camera_name);
		}
	}

	chai3d::cMultiMesh *SaiGraphics::createMultiMesh(const std::string &mesh_file_path,
													 const std::string &object_name,
													 const Eigen::Affine3d &object_pose,
													 const double size_factor)
	{
		// create a new multi-mesh
		auto multi_mesh = new chai3d::cMultiMesh();
		multi_mesh->m_name = object_name;
		_world->addChild(multi_mesh);
		// load the mesh file
		bool fileload = multi_mesh->loadFromFile(mesh_file_path);
		if (!fileload)
		{
			cout << "Error - Mesh file failed to load correctly." << endl;
		}
		// set the pose
		multi_mesh->setLocalPos(object_pose.translation());
		multi_mesh->setLocalRot(cMatrix3d(object_pose.rotation()));
		// apply uniform scale (size factor)
		multi_mesh->scale(size_factor);
		return multi_mesh;
	}

	void SaiGraphics::setCameraClippingPlanes(const double near_plane,
											  const double far_plane,
											  const std::string &camera_name)
	{
		if (camera_name == "default_camera")
		{
			// Iterate through the vector of all camera names in the world
			for (const std::string &name : _camera_names)
			{
				chai3d::cCamera *camera = getCamera(name);
				if (camera)
				{
					camera->setClippingPlanes(near_plane, far_plane);
				}
			}
		}
		else
		{
			// Apply only to the specific camera requested
			chai3d::cCamera *camera = getCamera(camera_name);
			if (camera)
			{
				camera->setClippingPlanes(near_plane, far_plane);
			}
			else
			{
				std::cerr << "Warning: Camera '" << camera_name << "' not found." << std::endl;
			}
		}
	}

	void SaiGraphics::setStereoMode(const bool stereo_enabled,
									const std::string &camera_name)
	{
		chai3d::cCamera *camera = getCamera(camera_name);
		if (stereo_enabled)
		{
			camera->setStereoMode(C_STEREO_PASSIVE_LEFT_RIGHT);
			camera->setStereoEyeSeparation(.02);
			camera->setStereoFocalLength(10.0);

			// chai3d::glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
		}
		else
		{
			camera->setStereoMode(C_STEREO_DISABLED);
		}
	}

	bool SaiGraphics::is_pressed(int key) const
	{
		if (key_presses_map.count(key) > 0)
			return key_presses_map.at(key).first;
		if (mouse_button_presses_map.count(key) > 0)
			return mouse_button_presses_map.at(key).first;
		return false;
	}

	bool SaiGraphics::key_pressed_once(int key)
	{
		if (key_presses_map.count(key) == 0)
			return false;

		auto &state = key_presses_map[key];
		if (state.first && !state.second)
		{
			state.second = true; // mark as consumed
			return true;
		}
		return false;
	}

	void SaiGraphics::createSidePanel(string main_camera_name, string side_camera_name)
	{
		auto side_camera = getCamera(side_camera_name);
		if (!side_camera)
			return;

		if (_camera_frame_buffers.find(side_camera_name) == _camera_frame_buffers.end())
		{
			_camera_frame_buffers[side_camera_name] = chai3d::cFrameBuffer::create();
		}
		auto frameBuffer = _camera_frame_buffers[side_camera_name];
		frameBuffer->setup(side_camera);

		auto borderPanel = new chai3d::cPanel();
		borderPanel->setColor(chai3d::cColorf(0.8f, 0.8f, 0.8f, 1.0f));
		borderPanel->setCornerRadius(20, 20, 20, 20);
		borderPanel->setTransparencyLevel(0.5f);

		auto sideViewPanel = new chai3d::cViewPanel(frameBuffer);
		sideViewPanel->setCornerRadius(18, 18, 18, 18);

		borderPanel->addChild(sideViewPanel);

		auto main_camera = getCamera(main_camera_name);
		main_camera->m_frontLayer->addChild(borderPanel);

		_active_side_panels[main_camera_name].push_back(frameBuffer);
	}

	chai3d::cGenericLight *SaiGraphics::getLight(const std::string &light_name)
	{
		for (unsigned int i = 0; i < _world->m_lights.size(); ++i)
		{
			auto light = _world->m_lights[i];
			if (light != nullptr && light->m_name == light_name)
			{
				return light;
			}
		}
		return nullptr;
	}

	std::vector<unsigned char> SaiGraphics::getFrameBuffer(const std::string &camera_name,
														   int width, int height)
	{
		// Ensures the camera exists in your graphics world
		auto it_cam = _camera_frame_buffers.find(camera_name);
		if (it_cam == _camera_frame_buffers.end())
		{
			throw std::invalid_argument("Camera not found: " + camera_name);
		}
		chai3d::cCamera *camera = it_cam->second->getCamera();

		// Initialization of the background frame buffer for this camera if it does not exist yet
		if (_camera_fbo_map.find(camera_name) == _camera_fbo_map.end())
		{
			auto fb = chai3d::cFrameBuffer::create();
			fb->setup(camera);
			fb->setSize(width, height);
			_camera_fbo_map[camera_name] = fb;
		}

		auto fb = _camera_fbo_map[camera_name];

		// Update size if requested dimensions changed
		if (fb->getWidth() != width || fb->getHeight() != height)
		{
			fb->setSize(width, height);
		}

		// Background Render and Capture
		fb->renderView();

		chai3d::cImagePtr image = chai3d::cImage::create();
		fb->copyImageBuffer(image);

		unsigned char *data = image->getData();
		unsigned int size = image->getSizeInBytes();

		return std::vector<unsigned char>(data, data + size);
	}

	chai3d::cLabel *SaiGraphics::addText(const std::string &camera_name, const std::string &text, double x, double y)
	{
		// Retrieve the target camera from the graphics world
		chai3d::cCamera *camera = getCamera(camera_name);

		// Match the warning convention used throughout SaiGraphics.cpp
		if (camera == nullptr)
		{
			std::cerr << "WARNING: Camera [" << camera_name << "] not found in graphics world. Cannot add text." << std::endl;
			return nullptr;
		}

		// Create a font using CHAI3D's built-in macro
		chai3d::cFontPtr font = chai3d::NEW_CFONTCALIBRI20();

		// Create a new label with the specified font
		chai3d::cLabel *label = new chai3d::cLabel(font);

		// Add the label to the camera's front layer so it acts as a 2D UI overlay
		camera->m_frontLayer->addChild(label);

		// Assign text color, value, and screen position
		label->m_fontColor.setBlack();
		label->setText(text);
		label->setLocalPos(x, y);

		return label;
	}

} // namespace SaiGraphics
