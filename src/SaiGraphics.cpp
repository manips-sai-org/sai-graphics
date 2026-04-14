/**
 * \file SaiGraphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "SaiGraphics.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

#ifdef MACOSX
#include <filesystem>
#endif

#include "parser/UrdfToSaiGraphics.h"

using namespace std;
using namespace chai3d;

namespace {
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
	{GLFW_KEY_LEFT_SHIFT, std::make_pair(false, true)},
	{GLFW_KEY_LEFT_ALT, std::make_pair(false, true)},
	{GLFW_KEY_LEFT_CONTROL, std::make_pair(false, true)},
};

std::unordered_map<int, std::pair<bool, bool>> mouse_button_presses_map = {
	{GLFW_MOUSE_BUTTON_LEFT, std::make_pair(false, true)},
	{GLFW_MOUSE_BUTTON_RIGHT, std::make_pair(false, true)},
	{GLFW_MOUSE_BUTTON_MIDDLE, std::make_pair(false, true)},
};

std::deque<double> mouse_scroll_buffer;

bool is_pressed(int key) {
	if (key_presses_map.count(key) > 0) {
		return key_presses_map.at(key).first;
	}
	if (mouse_button_presses_map.count(key) > 0) {
		return mouse_button_presses_map.at(key).first;
	}
	return false;
}

bool consume_first_press(int key) {
	if (key_presses_map.count(key) > 0) {
		if (key_presses_map.at(key).first && key_presses_map.at(key).second) {
			key_presses_map.at(key).second = false;
			return true;
		}
	}
	if (mouse_button_presses_map.count(key) > 0) {
		if (mouse_button_presses_map.at(key).first &&
			mouse_button_presses_map.at(key).second) {
			mouse_button_presses_map.at(key).second = false;
			return true;
		}
	}
	return false;
}

// callback to print glfw errors
void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action,
			   int mods) {
	bool set = (action != GLFW_RELEASE);
	if (key == GLFW_KEY_ESCAPE) {
		// handle esc separately to exit application
		glfwSetWindowShouldClose(window, GL_TRUE);
	} else {
		if (key_presses_map.count(key) > 0) {
			key_presses_map.at(key).first = set;
			if (!set) {
				key_presses_map.at(key).second = true;
			}
		}
	}
}

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	if (mouse_button_presses_map.count(button) > 0) {
		mouse_button_presses_map.at(button).first = set;
		if (!set) {
			mouse_button_presses_map.at(button).second = true;
		}
	}
}

// callback when the mouse wheel is scrolled
void mouseScroll(GLFWwindow* window, double xoffset, double yoffset) {
	if (yoffset != 0) {
		mouse_scroll_buffer.push_back(yoffset);
	} else if (xoffset != 0) {
		mouse_scroll_buffer.push_back(xoffset);
	}
}

GLFWwindow* glfwInitialize(const std::string& window_name) {
	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.5 * screenW;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = (screenW - windowW) / 2;

	// create window and make it current context
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window =
		glfwCreateWindow(windowW, windowH, window_name.c_str(), NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

enum class MuscleLimbGroup {
	kOther,
	kRightArm,
	kLeftArm,
	kRightLeg,
	kLeftLeg,
};

MuscleLimbGroup classifyMuscleLimbGroup(const SaiModel::MuscleNode& muscle) {
	bool uses_right_arm = false;
	bool uses_left_arm = false;
	bool uses_right_leg = false;
	bool uses_left_leg = false;

	for (const auto& waypoint : muscle.contractor.muscle_tendon_path) {
		const auto& link_name = waypoint.link_name;
		uses_right_arm = uses_right_arm ||
						 link_name.find("right_upper_arm") != std::string::npos ||
						 link_name.find("right_hand") != std::string::npos;
		uses_left_arm = uses_left_arm ||
						link_name.find("left_upper_arm") != std::string::npos ||
						link_name.find("left_hand") != std::string::npos;
		uses_right_leg = uses_right_leg ||
						 link_name.find("right_thigh") != std::string::npos ||
						 link_name.find("right_lower_leg") != std::string::npos ||
						 link_name.find("right_foot") != std::string::npos;
		uses_left_leg = uses_left_leg ||
						link_name.find("left_thigh") != std::string::npos ||
						link_name.find("left_lower_leg") != std::string::npos ||
						link_name.find("left_foot") != std::string::npos;
	}

	if (uses_right_arm) {
		return MuscleLimbGroup::kRightArm;
	}
	if (uses_left_arm) {
		return MuscleLimbGroup::kLeftArm;
	}
	if (uses_right_leg) {
		return MuscleLimbGroup::kRightLeg;
	}
	if (uses_left_leg) {
		return MuscleLimbGroup::kLeftLeg;
	}
	return MuscleLimbGroup::kOther;
}

chai3d::cColorf muscleLimbGroupColor(const MuscleLimbGroup group) {
	switch (group) {
		case MuscleLimbGroup::kRightArm:
			return chai3d::cColorf(0.86f, 0.27f, 0.23f);
		case MuscleLimbGroup::kLeftArm:
			return chai3d::cColorf(0.24f, 0.49f, 0.84f);
		case MuscleLimbGroup::kRightLeg:
			return chai3d::cColorf(0.96f, 0.63f, 0.17f);
		case MuscleLimbGroup::kLeftLeg:
			return chai3d::cColorf(0.22f, 0.67f, 0.42f);
		case MuscleLimbGroup::kOther:
		default:
			return chai3d::cColorf(0.53f, 0.31f, 0.78f);
	}
}

chai3d::cColorf muscleEndpointWaypointColor() {
	return chai3d::cColorf(0.16f, 0.72f, 0.29f);
}

chai3d::cColorf muscleIntermediateWaypointColor() {
	return chai3d::cColorf(0.93f, 0.82f, 0.18f);
}

chai3d::cColorf muscleTendonHighlightColor() {
	return chai3d::cColorf(0.98f, 0.97f, 0.92f);
}

double highlightedMuscleTendonLineWidth(const double base_line_width) {
	return std::max(4.0, base_line_width * 2.0);
}

bool projectWorldPointToViewport(chai3d::cCamera* camera,
								 const Eigen::Vector3d& world_point,
								 const int viewport_width,
								 const int viewport_height, double& out_x,
								 double& out_y, double& out_depth) {
	if (camera == nullptr || viewport_width <= 0 || viewport_height <= 0) {
		return false;
	}

	const double field_view_angle_deg = camera->getFieldViewAngleDeg();
	if (std::abs(field_view_angle_deg) < 1e-6) {
		return false;
	}

	const Eigen::Vector3d camera_delta =
		world_point - camera->getGlobalPos().eigen();
	const Eigen::Vector3d point_in_camera =
		camera->getGlobalRot().eigen().transpose() * camera_delta;

	// In CHAI3D camera coordinates, points in front of the camera have
	// negative x values.
	if (point_in_camera.x() >= -camera->getNearClippingPlane()) {
		return false;
	}

	const double dist_cam =
		(viewport_height / 2.0) / cTanDeg(field_view_angle_deg / 2.0);
	out_x =
		viewport_width / 2.0 - dist_cam * point_in_camera.y() / point_in_camera.x();
	out_y = viewport_height / 2.0 +
			dist_cam * point_in_camera.z() / point_in_camera.x();
	out_depth = -point_in_camera.x();
	return true;
}

double distanceToScreenSegment(const Eigen::Vector2d& point,
							   const Eigen::Vector2d& segment_start,
							   const Eigen::Vector2d& segment_end) {
	const Eigen::Vector2d segment = segment_end - segment_start;
	const double segment_length_squared = segment.squaredNorm();
	if (segment_length_squared <= 1e-9) {
		return (point - segment_start).norm();
	}

	const double projection =
		std::clamp((point - segment_start).dot(segment) / segment_length_squared,
				   0.0, 1.0);
	const Eigen::Vector2d closest_point =
		segment_start + projection * segment;
	return (point - closest_point).norm();
}

constexpr int kJointSliderMinVisibleRows = 8;
constexpr int kJointSliderHeaderHeightPx = 32;
constexpr int kJointSliderWindowWidthPx = 360;
constexpr int kJointSliderLeftPx = 18;
constexpr int kJointSliderTopPx = 18;
constexpr int kJointSliderBottomPx = 18;
constexpr int kJointSliderTrackLeftPx = 188;
constexpr int kJointSliderTrackRightPx = 332;
constexpr double kJointSliderDragThresholdPx = 12.0;
constexpr double kJointSliderMinRowHeightPx = 20.0;
constexpr double kJointSliderMaxRowHeightPx = 46.0;

double finiteSliderBound(const double value, const double fallback) {
	if (std::isfinite(value) && std::abs(value) < 1e6) {
		return value;
	}
	return fallback;
}

std::pair<double, double> sliderBoundsForJoint(
	const Eigen::VectorXd& q, const SaiModel::JointLimit& limit) {
	double lower = finiteSliderBound(limit.position_lower, -M_PI);
	double upper = finiteSliderBound(limit.position_upper, M_PI);
	if (upper < lower) {
		std::swap(lower, upper);
	}
	if (std::abs(upper - lower) < 1e-9) {
		const double center =
			(limit.joint_index >= 0 && limit.joint_index < q.size())
				? q(limit.joint_index)
				: 0.0;
		lower = center - 1.0;
		upper = center + 1.0;
	}
	return {lower, upper};
}

double clampSliderValue(const double value, const double lower,
						  const double upper) {
	return std::clamp(value, std::min(lower, upper), std::max(lower, upper));
}

std::string formatSliderValue(const double value) {
	std::ostringstream stream;
	stream << std::fixed << std::setprecision(3) << value;
	return stream.str();
}
}  // namespace

namespace SaiGraphics {

SaiGraphics::SaiGraphics(const std::string& path_to_world_file,
						   const std::string& window_name, bool verbose) {
	_joint_slider_dropdown_enabled = false;
	_joint_slider_dropdown_expanded = false;
	_joint_slider_scroll_index = 0;
	_active_joint_slider_index = -1;
	_joint_slider_interaction_occurring = false;
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
SaiGraphics::~SaiGraphics() {
	glfwDestroyWindow(_window);
	glfwTerminate();
	clearWorld();
	_world = NULL;
}

void SaiGraphics::resetWorld(const std::string& path_to_world_file,
							  const bool verbose) {
	clearWorld();
	initializeWorld(path_to_world_file, verbose);
}

void SaiGraphics::initializeWorld(const std::string& path_to_world_file,
								   const bool verbose) {
	_world = new chai3d::cWorld();
	Parser::UrdfToSaiGraphicsWorld(
		path_to_world_file, _world, _robot_filenames, _dyn_objects_pose,
		_static_objects_pose, _camera_frame_buffers, verbose);
	_current_camera_index = 0;
	for (auto it : _camera_frame_buffers) {
		_camera_names.push_back(it.first);
	}
	initializeMuscleTendonHoverLabels();
	initializeJointFrameHoverLabels();
	initializeJointSliderDropdowns();
	for (auto robot_filename : _robot_filenames) {
		// get robot base object in chai world
		cRobotBase* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_filename.first == _world->getChild(i)->m_name) {
				// cast to cRobotBase
				base = dynamic_cast<cRobotBase*>(_world->getChild(i));
				if (base != NULL) {
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
		_joint_slider_values[robot_filename.first] =
			_robot_models[robot_filename.first]->q();  
		_joint_slider_default_values[robot_filename.first] =
			_robot_models[robot_filename.first]->q() * 0; // hard-code to zero
		_joint_slider_override_active[robot_filename.first] = false;
		updateRobotGraphics(robot_filename.first,
							_robot_models[robot_filename.first]->q());
	}
	if (_joint_slider_robot_name.empty() && !_robot_filenames.empty()) {
		_joint_slider_robot_name = _robot_filenames.begin()->first;
	}
	for (auto object_pose : _dyn_objects_pose) {
		_object_velocities[object_pose.first] =
			std::make_shared<Eigen::Vector6d>(Eigen::Vector6d::Zero());
	}
	_right_click_interaction_occurring = false;
}

void SaiGraphics::clearWorld() {
	delete _world;
	_robot_filenames.clear();
	_robot_models.clear();
	_dyn_objects_pose.clear();
	_static_objects_pose.clear();
	_object_velocities.clear();
	_camera_names.clear();
	_camera_frame_buffers.clear();
	_force_sensor_displays.clear();
	_muscle_tendon_path_lines.clear();
	_muscle_tendon_waypoints.clear();
	_muscle_tendon_hover_labels.clear();
	_muscle_tendon_hover_font.reset();
	_joint_frame_hover_labels.clear();
	_joint_frame_hover_font.reset();
	_joint_frame_displays.clear();
	_joint_slider_dropdowns.clear();
	_joint_slider_font.reset();
	_joint_slider_values.clear();
	_joint_slider_default_values.clear();
	_joint_slider_override_active.clear();
	_joint_slider_robot_name.clear();
	_joint_slider_scroll_index = 0;
	_active_joint_slider_index = -1;
	_joint_slider_interaction_occurring = false;
	_ui_force_widgets.clear();
	_camera_link_attachments.clear();
}

void SaiGraphics::initializeMuscleTendonHoverLabels() {
	_muscle_tendon_hover_font = NEW_CFONTCALIBRI72();
	for (const auto& camera_name : _camera_names) {
		auto* hover_label = new chai3d::cLabel(_muscle_tendon_hover_font);
		hover_label->setFontScale(1.0);
		hover_label->m_fontColor.setRedCrimson();
		hover_label->setShowEnabled(false);
		getCamera(camera_name)->m_frontLayer->addChild(hover_label);
		_muscle_tendon_hover_labels[camera_name] = hover_label;
	}
}

void SaiGraphics::initializeJointFrameHoverLabels() {
	_joint_frame_hover_font = NEW_CFONTCALIBRI72();
	for (const auto& camera_name : _camera_names) {
		auto* hover_label = new chai3d::cLabel(_joint_frame_hover_font);
		hover_label->setFontScale(1.0);
		hover_label->m_fontColor.setWhite();
		hover_label->setShowEnabled(false);
		getCamera(camera_name)->m_frontLayer->addChild(hover_label);
		_joint_frame_hover_labels[camera_name] = hover_label;
	}
}

void SaiGraphics::initializeJointSliderDropdowns() {
	_joint_slider_font = NEW_CFONTCALIBRI72();
	for (const auto& camera_name : _camera_names) {
		JointSliderDropdownVisual dropdown = {};
		auto* front_layer = getCamera(camera_name)->m_frontLayer;

		dropdown.title_label = new chai3d::cLabel(_joint_slider_font);
		dropdown.title_label->setFontScale(0.9);
		dropdown.title_label->m_fontColor.setWhite();
		front_layer->addChild(dropdown.title_label);

		dropdown.state_label = new chai3d::cLabel(_joint_slider_font);
		dropdown.state_label->setFontScale(0.8);
		dropdown.state_label->m_fontColor.setGrayGainsboro();
		front_layer->addChild(dropdown.state_label);

		dropdown.reset_label = new chai3d::cLabel(_joint_slider_font);
		dropdown.reset_label->setFontScale(0.7);
		dropdown.reset_label->m_fontColor.setWhite();
		front_layer->addChild(dropdown.reset_label);

		dropdown.top_border = new chai3d::cShapeLine();
		dropdown.bottom_border = new chai3d::cShapeLine();
		dropdown.left_border = new chai3d::cShapeLine();
		dropdown.right_border = new chai3d::cShapeLine();
		dropdown.separator_line = new chai3d::cShapeLine();
		dropdown.reset_top_border = new chai3d::cShapeLine();
		dropdown.reset_bottom_border = new chai3d::cShapeLine();
		dropdown.reset_left_border = new chai3d::cShapeLine();
		dropdown.reset_right_border = new chai3d::cShapeLine();
		for (auto* line : {dropdown.top_border, dropdown.bottom_border,
						   dropdown.left_border, dropdown.right_border,
						   dropdown.separator_line, dropdown.reset_top_border,
						   dropdown.reset_bottom_border,
						   dropdown.reset_left_border,
						   dropdown.reset_right_border}) {
			line->setLineWidth(2.0);
			line->m_colorPointA = chai3d::cColorf(0.82f, 0.85f, 0.88f);
			line->m_colorPointB = chai3d::cColorf(0.82f, 0.85f, 0.88f);
			front_layer->addChild(line);
		}

		dropdown.rows.reserve(kJointSliderMinVisibleRows);
		for (int i = 0; i < kJointSliderMinVisibleRows; ++i) {
			JointSliderRowVisual row = {};
			row.name_label = new chai3d::cLabel(_joint_slider_font);
			row.name_label->setFontScale(0.72);
			row.name_label->m_fontColor.setWhite();
			front_layer->addChild(row.name_label);

			row.value_label = new chai3d::cLabel(_joint_slider_font);
			row.value_label->setFontScale(0.68);
			row.value_label->m_fontColor.setGrayGainsboro();
			front_layer->addChild(row.value_label);

			row.track_line = new chai3d::cShapeLine();
			row.fill_line = new chai3d::cShapeLine();
			row.handle_line = new chai3d::cShapeLine();
			row.track_line->setLineWidth(2.0);
			row.fill_line->setLineWidth(3.0);
			row.handle_line->setLineWidth(4.0);
			row.track_line->m_colorPointA = chai3d::cColorf(0.38f, 0.41f, 0.46f);
			row.track_line->m_colorPointB = chai3d::cColorf(0.38f, 0.41f, 0.46f);
			row.fill_line->m_colorPointA = chai3d::cColorf(0.30f, 0.69f, 0.95f);
			row.fill_line->m_colorPointB = chai3d::cColorf(0.30f, 0.69f, 0.95f);
			row.handle_line->m_colorPointA = chai3d::cColorf(0.95f, 0.96f, 0.98f);
			row.handle_line->m_colorPointB = chai3d::cColorf(0.95f, 0.96f, 0.98f);
			front_layer->addChild(row.track_line);
			front_layer->addChild(row.fill_line);
			front_layer->addChild(row.handle_line);

			dropdown.rows.push_back(row);
		}

		_joint_slider_dropdowns[camera_name] = dropdown;
	}
}

void SaiGraphics::syncJointSliderDropdownState() {
	if (_joint_slider_robot_name.empty() && !_robot_models.empty()) {
		_joint_slider_robot_name = _robot_models.begin()->first;
	}
	for (const auto& robot_entry : _robot_models) {
		const auto& robot_name = robot_entry.first;
		const auto& robot_model = robot_entry.second;
		if (_joint_slider_values.find(robot_name) == _joint_slider_values.end()) {
			_joint_slider_values[robot_name] = robot_model->q();
		}
		if (_joint_slider_override_active.find(robot_name) ==
			_joint_slider_override_active.end()) {
			_joint_slider_override_active[robot_name] = false;
		}
		if (!_joint_slider_override_active[robot_name]) {
			_joint_slider_values[robot_name] = robot_model->q();
		}
	}
	if (_joint_slider_robot_name.empty() ||
		_robot_models.find(_joint_slider_robot_name) == _robot_models.end()) {
		_joint_slider_robot_name =
			_robot_models.empty() ? "" : _robot_models.begin()->first;
	}
	if (_joint_slider_robot_name.empty()) {
		_joint_slider_scroll_index = 0;
		return;
	}
	const int joint_count = static_cast<int>(
		_robot_models.at(_joint_slider_robot_name)->jointLimits().size());
	const int available_height =
		std::max(0, _window_height - kJointSliderTopPx - kJointSliderBottomPx -
						  kJointSliderHeaderHeightPx);
	const int max_visible_rows = std::max(
		1, static_cast<int>(std::floor(
			   available_height / kJointSliderMinRowHeightPx)));
	_joint_slider_scroll_index =
		std::clamp(_joint_slider_scroll_index, 0,
				   std::max(0, joint_count - max_visible_rows));
}

void SaiGraphics::showJointPositionSliders(bool show,
											 const std::string& robot_name) {
	_joint_slider_dropdown_enabled = show;
	_joint_slider_dropdown_expanded = show;
	_joint_slider_interaction_occurring = false;
	_active_joint_slider_index = -1;

	if (!robot_name.empty()) {
		if (!robotExistsInWorld(robot_name)) {
			throw std::invalid_argument(
				"robot not found in SaiGraphics::showJointPositionSliders");
		}
		_joint_slider_robot_name = robot_name;
	} else if (_joint_slider_robot_name.empty() && !_robot_models.empty()) {
		_joint_slider_robot_name = _robot_models.begin()->first;
	}

	syncJointSliderDropdownState();
}

void SaiGraphics::updateMuscleTendonHoverLabel(const std::string& camera_name,
												 const double cursorx,
												 const double cursory,
												 const int window_width_screen,
												 const int window_height_screen) {
	auto label_it = _muscle_tendon_hover_labels.find(camera_name);
	if (label_it == _muscle_tendon_hover_labels.end()) {
		return;
	}

	auto* hover_label = label_it->second;
	hover_label->setShowEnabled(false);
	std::string hovered_robot_name;
	std::string hovered_muscle_name;

	if (!_muscle_tendon_waypoints.empty() && window_width_screen > 0 &&
		window_height_screen > 0) {
		const int viewx = floor(cursorx / window_width_screen * _window_width);
		const int viewy = floor(cursory / window_height_screen * _window_height);

		chai3d::cCollisionRecorder collision_recorder;
		chai3d::cCollisionSettings collision_settings;
		const bool hit = getCamera(camera_name)->selectWorld(
			viewx, _window_height - viewy, _window_width, _window_height,
			collision_recorder, collision_settings);
		if (hit) {
			const chai3d::cGenericObject* hit_object =
				collision_recorder.m_nearestCollision.m_object;
			for (const auto& waypoint_display : _muscle_tendon_waypoints) {
				if (waypoint_display.sphere != hit_object) {
					continue;
				}

				hovered_robot_name = waypoint_display.robot_name;
				hovered_muscle_name = waypoint_display.muscle_name;
				hover_label->setText(waypoint_display.muscle_name);
				const int label_x =
					std::min(std::max(0, viewx + 14),
							 std::max(0, _window_width -
											static_cast<int>(hover_label->getWidth())));
				const int label_y =
					std::min(std::max(0, _window_height - viewy + 18),
							 std::max(0, _window_height - static_cast<int>(
														 hover_label->getHeight())));
				hover_label->setLocalPos(label_x, label_y, 0);
				hover_label->setShowEnabled(true);
				break;
			}
		}
	}

	updateMuscleTendonPathHighlight(hovered_robot_name, hovered_muscle_name);
}

void SaiGraphics::updateMuscleTendonPathHighlight(
	const std::string& robot_name, const std::string& muscle_name) {
	const bool has_hovered_muscle = !robot_name.empty() && !muscle_name.empty();
	const auto highlight_color = muscleTendonHighlightColor();
	for (auto& segment : _muscle_tendon_path_lines) {
		const bool is_hovered_segment =
			has_hovered_muscle && segment.robot_name == robot_name &&
			segment.muscle_name == muscle_name;
		const auto color = is_hovered_segment ? highlight_color : segment.color;
		segment.line->m_colorPointA = color;
		segment.line->m_colorPointB = color;
		segment.line->setLineWidth(
			is_hovered_segment
				? highlightedMuscleTendonLineWidth(segment.line_width)
				: segment.line_width);
	}
}

void SaiGraphics::updateJointFrameHoverLabel(const std::string& camera_name,
											 const double cursorx,
											 const double cursory,
											 const int window_width_screen,
											 const int window_height_screen) {
	auto label_it = _joint_frame_hover_labels.find(camera_name);
	if (label_it == _joint_frame_hover_labels.end()) {
		return;
	}

	auto* hover_label = label_it->second;
	hover_label->setShowEnabled(false);
	if (_joint_frame_displays.empty() || window_width_screen <= 0 ||
		window_height_screen <= 0) {
		return;
	}

	const int viewx = floor(cursorx / window_width_screen * _window_width);
	const int viewy = floor(cursory / window_height_screen * _window_height);
	auto* camera = getCamera(camera_name);
	_world->computeGlobalPositions(false);

	const Eigen::Vector2d cursor_position(viewx, viewy);
	const JointFrameDisplay* hovered_joint_frame = nullptr;
	double best_distance = std::numeric_limits<double>::infinity();

	for (const auto& joint_frame_display : _joint_frame_displays) {
		if (joint_frame_display.link == nullptr ||
			!joint_frame_display.link->getShowFrame()) {
			continue;
		}

		const Eigen::Vector3d origin =
			joint_frame_display.link->getGlobalPos().eigen();
		const Eigen::Matrix3d rotation =
			joint_frame_display.link->getGlobalRot().eigen();

		double origin_x = 0.0;
		double origin_y = 0.0;
		double origin_depth = 0.0;
		if (!projectWorldPointToViewport(camera, origin, _window_width,
										 _window_height, origin_x, origin_y,
										 origin_depth)) {
			continue;
		}

		double joint_distance =
			(cursor_position - Eigen::Vector2d(origin_x, origin_y)).norm();
		for (int axis = 0; axis < 3; ++axis) {
			const Eigen::Vector3d axis_endpoint =
				origin + rotation.col(axis) *
							 joint_frame_display.frame_pointer_length;
			double axis_x = 0.0;
			double axis_y = 0.0;
			double axis_depth = 0.0;
			if (!projectWorldPointToViewport(camera, axis_endpoint, _window_width,
											 _window_height, axis_x, axis_y,
											 axis_depth)) {
				continue;
			}
			joint_distance = std::min(
				joint_distance,
				distanceToScreenSegment(cursor_position,
										Eigen::Vector2d(origin_x, origin_y),
										Eigen::Vector2d(axis_x, axis_y)));
		}

		const double hover_threshold =
			std::max(10.0, 0.02 * _window_height + 8.0 / origin_depth);
		if (joint_distance <= hover_threshold && joint_distance < best_distance) {
			best_distance = joint_distance;
			hovered_joint_frame = &joint_frame_display;
		}
	}

	if (hovered_joint_frame == nullptr) {
		return;
	}

	hover_label->setText(hovered_joint_frame->joint_name);
	const int label_x =
		std::min(std::max(0, viewx + 14),
				 std::max(0, _window_width -
									static_cast<int>(hover_label->getWidth())));
	const int label_y = std::min(
		std::max(0, _window_height - viewy + 18),
		std::max(0, _window_height -
						 static_cast<int>(hover_label->getHeight())));
	hover_label->setLocalPos(label_x, label_y, 0);
	hover_label->setShowEnabled(true);
}

bool SaiGraphics::updateJointSliderDropdown(const std::string& camera_name,
											   const double cursorx,
											   const double cursory,
											   const int window_width_screen,
											   const int window_height_screen,
											   const double scroll_value) {
	auto dropdown_it = _joint_slider_dropdowns.find(camera_name);
	if (dropdown_it == _joint_slider_dropdowns.end()) {
		return false;
	}

	auto& dropdown = dropdown_it->second;
	const auto set_dropdown_visible = [&dropdown](const bool show_header,
												 const bool show_rows) {
		dropdown.title_label->setShowEnabled(show_header);
		dropdown.state_label->setShowEnabled(show_header);
		dropdown.reset_label->setShowEnabled(show_header);
		for (auto* border : {dropdown.top_border, dropdown.bottom_border,
							 dropdown.left_border, dropdown.right_border,
							 dropdown.separator_line, dropdown.reset_top_border,
							 dropdown.reset_bottom_border,
							 dropdown.reset_left_border,
							 dropdown.reset_right_border}) {
			border->setShowEnabled(show_header);
		}
		for (auto& row : dropdown.rows) {
			row.name_label->setShowEnabled(show_rows);
			row.value_label->setShowEnabled(show_rows);
			row.track_line->setShowEnabled(show_rows);
			row.fill_line->setShowEnabled(show_rows);
			row.handle_line->setShowEnabled(show_rows);
		}
	};

	syncJointSliderDropdownState();
	if (!_joint_slider_dropdown_enabled || _joint_slider_robot_name.empty()) {
		set_dropdown_visible(false, false);
		return false;
	}

	auto robot_it = _robot_models.find(_joint_slider_robot_name);
	if (robot_it == _robot_models.end()) {
		set_dropdown_visible(false, false);
		return false;
	}

	const auto& robot_model = robot_it->second;
	const auto& joint_limits = robot_model->jointLimits();
	const auto values_it = _joint_slider_values.find(_joint_slider_robot_name);
	if (values_it == _joint_slider_values.end()) {
		set_dropdown_visible(false, false);
		return false;
	}
	const auto& slider_values = values_it->second;

	const int panel_x = kJointSliderLeftPx;
	const int panel_y = kJointSliderTopPx;
	const int header_top = panel_y;
	const int header_bottom = panel_y + kJointSliderHeaderHeightPx;
	const int expanded_panel_bottom =
		std::max(header_bottom, _window_height - kJointSliderBottomPx);
	const int available_rows_height =
		std::max(0, expanded_panel_bottom - header_bottom);
	const int max_visible_rows = std::max(
		1, static_cast<int>(std::floor(
			   available_rows_height / kJointSliderMinRowHeightPx)));
	const int visible_rows = _joint_slider_dropdown_expanded
								 ? std::min(max_visible_rows,
											static_cast<int>(joint_limits.size()))
								 : 0;
	const double raw_row_height =
		visible_rows > 0
			? available_rows_height / static_cast<double>(visible_rows)
			: kJointSliderMinRowHeightPx;
	const double row_height =
		std::clamp(raw_row_height, kJointSliderMinRowHeightPx,
				   kJointSliderMaxRowHeightPx);
	const int panel_bottom = _joint_slider_dropdown_expanded
								 ? expanded_panel_bottom
								 : header_bottom;
	const auto local_y = [this](const double top_origin_y) {
		return _window_height - top_origin_y;
	};
	const double title_font_scale =
		std::clamp(0.3 + row_height / 30.0, 0.30, 0.40);
	const double subtitle_font_scale =
		std::clamp(0.3 + row_height / 34.0, 0.30, 0.40);
	const double row_font_scale =
		std::clamp(0.3 + row_height / 24.0, 0.38, 0.40);
	const double value_font_scale =
		std::clamp(0.3 + row_height / 26.0, 0.3, 0.40);
	const double button_font_scale =
		std::clamp(0.3 + row_height / 28.0, 0.3, 0.40);
	const int reset_button_width = 74;
	const int reset_button_height = 20;
	const int reset_button_right = panel_x + kJointSliderWindowWidthPx - 10;
	const int reset_button_left = reset_button_right - reset_button_width;
	const int reset_button_top = panel_y + 6;
	const int reset_button_bottom = reset_button_top + reset_button_height;

	dropdown.title_label->setText("Joint Controls");
	dropdown.title_label->setFontScale(title_font_scale);
	dropdown.title_label->setLocalPos(panel_x + 10, local_y(panel_y + 18), 0);
	dropdown.state_label->setText(
		_joint_slider_robot_name +
		std::string(_joint_slider_dropdown_expanded ? "  v" : "  >"));
	dropdown.state_label->setFontScale(subtitle_font_scale);
	dropdown.state_label->setLocalPos(panel_x + 150, local_y(panel_y + 18), 0);
	dropdown.reset_label->setText("Reset");
	dropdown.reset_label->setFontScale(button_font_scale);
	dropdown.reset_label->setLocalPos(reset_button_left + 14,
									  local_y(reset_button_top + 14), 0);
	dropdown.reset_label->setShowEnabled(true);

	dropdown.top_border->m_pointA =
		chai3d::cVector3d(panel_x, local_y(header_top), 0);
	dropdown.top_border->m_pointB = chai3d::cVector3d(
		panel_x + kJointSliderWindowWidthPx, local_y(header_top), 0);
	dropdown.bottom_border->m_pointA =
		chai3d::cVector3d(panel_x, local_y(panel_bottom), 0);
	dropdown.bottom_border->m_pointB = chai3d::cVector3d(
		panel_x + kJointSliderWindowWidthPx, local_y(panel_bottom), 0);
	dropdown.left_border->m_pointA =
		chai3d::cVector3d(panel_x, local_y(header_top), 0);
	dropdown.left_border->m_pointB =
		chai3d::cVector3d(panel_x, local_y(panel_bottom), 0);
	dropdown.right_border->m_pointA = chai3d::cVector3d(
		panel_x + kJointSliderWindowWidthPx, local_y(header_top), 0);
	dropdown.right_border->m_pointB = chai3d::cVector3d(
		panel_x + kJointSliderWindowWidthPx, local_y(panel_bottom), 0);
	dropdown.separator_line->m_pointA =
		chai3d::cVector3d(panel_x, local_y(header_bottom), 0);
	dropdown.separator_line->m_pointB = chai3d::cVector3d(
		panel_x + kJointSliderWindowWidthPx, local_y(header_bottom), 0);
	dropdown.reset_top_border->m_pointA =
		chai3d::cVector3d(reset_button_left, local_y(reset_button_top), 0);
	dropdown.reset_top_border->m_pointB =
		chai3d::cVector3d(reset_button_right, local_y(reset_button_top), 0);
	dropdown.reset_bottom_border->m_pointA =
		chai3d::cVector3d(reset_button_left, local_y(reset_button_bottom), 0);
	dropdown.reset_bottom_border->m_pointB =
		chai3d::cVector3d(reset_button_right, local_y(reset_button_bottom), 0);
	dropdown.reset_left_border->m_pointA =
		chai3d::cVector3d(reset_button_left, local_y(reset_button_top), 0);
	dropdown.reset_left_border->m_pointB =
		chai3d::cVector3d(reset_button_left, local_y(reset_button_bottom), 0);
	dropdown.reset_right_border->m_pointA =
		chai3d::cVector3d(reset_button_right, local_y(reset_button_top), 0);
	dropdown.reset_right_border->m_pointB =
		chai3d::cVector3d(reset_button_right, local_y(reset_button_bottom), 0);

	set_dropdown_visible(true, _joint_slider_dropdown_expanded);

	const bool cursor_valid =
		window_width_screen > 0 && window_height_screen > 0 && _window_width > 0 &&
		_window_height > 0;
	const double viewx =
		cursor_valid ? cursorx / window_width_screen * _window_width : -1.0;
	const double viewy =
		cursor_valid ? cursory / window_height_screen * _window_height : -1.0;

	const bool cursor_on_header =
		cursor_valid && viewx >= panel_x &&
		viewx <= panel_x + kJointSliderWindowWidthPx && viewy >= header_top &&
		viewy <= header_bottom;
	const bool cursor_on_reset =
		cursor_valid && viewx >= reset_button_left &&
		viewx <= reset_button_right && viewy >= reset_button_top &&
		viewy <= reset_button_bottom;
	const bool cursor_in_panel =
		cursor_valid && viewx >= panel_x &&
		viewx <= panel_x + kJointSliderWindowWidthPx && viewy >= header_top &&
		viewy <= panel_bottom;
	const bool left_mouse_first_press =
		is_pressed(GLFW_MOUSE_BUTTON_LEFT) &&
		mouse_button_presses_map.at(GLFW_MOUSE_BUTTON_LEFT).second;

	bool consumed = false;
	if (left_mouse_first_press && cursor_on_reset) {
		mouse_button_presses_map.at(GLFW_MOUSE_BUTTON_LEFT).second = false;
		auto default_it =
			_joint_slider_default_values.find(_joint_slider_robot_name);
		if (default_it != _joint_slider_default_values.end()) {
			_joint_slider_values[_joint_slider_robot_name] = default_it->second;
			_joint_slider_override_active[_joint_slider_robot_name] = true;
		}
		_active_joint_slider_index = -1;
		_joint_slider_interaction_occurring = false;
		consumed = true;
	}
	if (!consumed && left_mouse_first_press && cursor_on_header) {
		mouse_button_presses_map.at(GLFW_MOUSE_BUTTON_LEFT).second = false;
		_joint_slider_dropdown_expanded = !_joint_slider_dropdown_expanded;
		_active_joint_slider_index = -1;
		_joint_slider_interaction_occurring = false;
		consumed = true;
	}

	if (_joint_slider_dropdown_expanded && cursor_in_panel &&
		std::abs(scroll_value) > 1e-9 &&
		joint_limits.size() > static_cast<size_t>(max_visible_rows)) {
		const int direction = (scroll_value > 0.0) ? -1 : 1;
		_joint_slider_scroll_index = std::clamp(
			_joint_slider_scroll_index + direction, 0,
			std::max(0, static_cast<int>(joint_limits.size()) -
							 max_visible_rows));
		consumed = true;
	}

	if (!is_pressed(GLFW_MOUSE_BUTTON_LEFT)) {
		_active_joint_slider_index = -1;
		_joint_slider_interaction_occurring = false;
	}

	while (static_cast<int>(dropdown.rows.size()) < visible_rows) {
		JointSliderRowVisual row = {};
		auto* front_layer = getCamera(camera_name)->m_frontLayer;
		row.name_label = new chai3d::cLabel(_joint_slider_font);
		row.name_label->m_fontColor.setWhite();
		front_layer->addChild(row.name_label);

		row.value_label = new chai3d::cLabel(_joint_slider_font);
		row.value_label->m_fontColor.setGrayGainsboro();
		front_layer->addChild(row.value_label);

		row.track_line = new chai3d::cShapeLine();
		row.fill_line = new chai3d::cShapeLine();
		row.handle_line = new chai3d::cShapeLine();
		row.track_line->m_colorPointA = chai3d::cColorf(0.38f, 0.41f, 0.46f);
		row.track_line->m_colorPointB = chai3d::cColorf(0.38f, 0.41f, 0.46f);
		row.fill_line->m_colorPointA = chai3d::cColorf(0.30f, 0.69f, 0.95f);
		row.fill_line->m_colorPointB = chai3d::cColorf(0.30f, 0.69f, 0.95f);
		row.handle_line->m_colorPointA = chai3d::cColorf(0.95f, 0.96f, 0.98f);
		row.handle_line->m_colorPointB = chai3d::cColorf(0.95f, 0.96f, 0.98f);
		front_layer->addChild(row.track_line);
		front_layer->addChild(row.fill_line);
		front_layer->addChild(row.handle_line);
		dropdown.rows.push_back(row);
	}

	for (int row_index = 0; row_index < static_cast<int>(dropdown.rows.size());
		 ++row_index) {
		auto& row = dropdown.rows[row_index];
		if (!_joint_slider_dropdown_expanded ||
			row_index >= visible_rows ||
			row_index >= static_cast<int>(joint_limits.size()) -
							_joint_slider_scroll_index) {
			row.name_label->setShowEnabled(false);
			row.value_label->setShowEnabled(false);
			row.track_line->setShowEnabled(false);
			row.fill_line->setShowEnabled(false);
			row.handle_line->setShowEnabled(false);
			continue;
		}

		const int joint_limit_index = _joint_slider_scroll_index + row_index;
		if (joint_limit_index >= static_cast<int>(joint_limits.size())) {
			row.name_label->setShowEnabled(false);
			row.value_label->setShowEnabled(false);
			row.track_line->setShowEnabled(false);
			row.fill_line->setShowEnabled(false);
			row.handle_line->setShowEnabled(false);
			continue;
		}

		const auto& limit = joint_limits[joint_limit_index];
		if (limit.joint_index < 0 || limit.joint_index >= slider_values.size()) {
			row.name_label->setShowEnabled(false);
			row.value_label->setShowEnabled(false);
			row.track_line->setShowEnabled(false);
			row.fill_line->setShowEnabled(false);
			row.handle_line->setShowEnabled(false);
			continue;
		}

		const double dynamic_row_top = header_bottom + row_index * row_height;
		const double dynamic_row_center_y = dynamic_row_top + 0.5 * row_height;
		const int row_label_y = local_y(dynamic_row_center_y + 0.22 * row_height);
		const auto [lower, upper] = sliderBoundsForJoint(slider_values, limit);
		const double clamped_value =
			clampSliderValue(slider_values(limit.joint_index), lower, upper);
		const double slider_alpha =
			(clamped_value - lower) / std::max(1e-9, upper - lower);
		const double slider_x = kJointSliderTrackLeftPx +
								slider_alpha *
									(kJointSliderTrackRightPx - kJointSliderTrackLeftPx);

		row.name_label->setText(limit.joint_name);
		row.name_label->setFontScale(row_font_scale);
		row.name_label->setLocalPos(panel_x + 10, row_label_y, 0);
		row.name_label->setShowEnabled(true);

		row.value_label->setText(formatSliderValue(clamped_value));
		row.value_label->setFontScale(value_font_scale);
		row.value_label->setLocalPos(panel_x + kJointSliderTrackRightPx + 10,
									 row_label_y, 0);
		row.value_label->setShowEnabled(true);

		const double track_width =
			std::clamp(0.10 * row_height, 2.0, 5.0);
		const double fill_width =
			std::clamp(0.15 * row_height, 3.0, 6.0);
		const double handle_width =
			std::clamp(0.18 * row_height, 4.0, 7.0);
		const double handle_half_height =
			std::clamp(0.33 * row_height, 7.0, 14.0);
		row.track_line->setLineWidth(track_width);
		row.fill_line->setLineWidth(fill_width);
		row.handle_line->setLineWidth(handle_width);

		row.track_line->m_pointA = chai3d::cVector3d(panel_x + kJointSliderTrackLeftPx,
												local_y(dynamic_row_center_y), 0);
		row.track_line->m_pointB = chai3d::cVector3d(panel_x + kJointSliderTrackRightPx,
												local_y(dynamic_row_center_y), 0);
		row.track_line->setShowEnabled(true);

		row.fill_line->m_pointA = row.track_line->m_pointA;
		row.fill_line->m_pointB =
			chai3d::cVector3d(panel_x + slider_x, local_y(dynamic_row_center_y), 0);
		row.fill_line->setShowEnabled(true);

		row.handle_line->m_pointA =
			chai3d::cVector3d(panel_x + slider_x,
							  local_y(dynamic_row_center_y - handle_half_height), 0);
		row.handle_line->m_pointB =
			chai3d::cVector3d(panel_x + slider_x,
							  local_y(dynamic_row_center_y + handle_half_height), 0);
		row.handle_line->setShowEnabled(true);

		if (!cursor_valid) {
			continue;
		}

		const bool cursor_on_slider =
			viewx >= panel_x + kJointSliderTrackLeftPx - 8 &&
			viewx <= panel_x + kJointSliderTrackRightPx + 8 &&
			std::abs(viewy - dynamic_row_center_y) <=
				std::max(kJointSliderDragThresholdPx, 0.40 * row_height);

		if (left_mouse_first_press && cursor_on_slider) {
			mouse_button_presses_map.at(GLFW_MOUSE_BUTTON_LEFT).second = false;
			_active_joint_slider_index = limit.joint_index;
			_joint_slider_interaction_occurring = true;
			consumed = true;
		}

		if (_active_joint_slider_index == limit.joint_index &&
			is_pressed(GLFW_MOUSE_BUTTON_LEFT)) {
			const double alpha = std::clamp(
				(viewx - (panel_x + kJointSliderTrackLeftPx)) /
					static_cast<double>(kJointSliderTrackRightPx -
									   kJointSliderTrackLeftPx),
				0.0, 1.0);
			_joint_slider_values[_joint_slider_robot_name](limit.joint_index) =
				lower + alpha * (upper - lower);
			_joint_slider_override_active[_joint_slider_robot_name] = true;
			_joint_slider_interaction_occurring = true;
			consumed = true;
		}
	}

	return consumed || cursor_in_panel || _joint_slider_interaction_occurring;
}

void SaiGraphics::applyJointSliderOverrides() {
	syncJointSliderDropdownState();
	for (const auto& override_entry : _joint_slider_override_active) {
		if (!override_entry.second) {
			continue;
		}
		const auto robot_it = _robot_models.find(override_entry.first);
		const auto value_it = _joint_slider_values.find(override_entry.first);
		if (robot_it == _robot_models.end() || value_it == _joint_slider_values.end()) {
			continue;
		}
		updateRobotGraphics(override_entry.first, value_it->second,
							robot_it->second->dq());
	}
}

void SaiGraphics::initializeWindow(const std::string& window_name) {
	_window = glfwInitialize(window_name);

	// set callbacks
	glfwSetKeyCallback(_window, keySelect);
	glfwSetMouseButtonCallback(_window, mouseClick);
	glfwSetScrollCallback(_window, mouseScroll);
}

void SaiGraphics::setCameraPose(const std::string& camera_name,
								 const Eigen::Affine3d& camera_pose) {
	if (!cameraExistsInWorld(camera_name)) {
		cout << "WARNING: Camera [" << camera_name
			 << "] does not exists in the graphics world. Cannot set pose"
			 << endl;
		return;
	}
	if (_camera_link_attachments.find(camera_name) ==
		_camera_link_attachments.end()) {
		cout << "WARNING: Cannot set pose for camera [" << camera_name
			 << "] attached to a robot or object" << endl;
		return;
	}
	Vector3d pos = camera_pose.translation();
	Vector3d up = -camera_pose.rotation().col(1);
	Vector3d lookat = pos + camera_pose.linear().col(2);
	setCameraPoseInternal(camera_name, pos, up, lookat);
}

Eigen::Affine3d SaiGraphics::getCameraPose(const std::string& camera_name) {
	if (!cameraExistsInWorld(camera_name)) {
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
	const std::string& camera_name, const std::string& robot_name,
	const std::string& link_name, const Eigen::Affine3d& pose_in_link) {
	if (!cameraExistsInWorld(camera_name)) {
		cout << "WARNING: camera [" << camera_name
			 << "] not found in graphics world, cannot attach to robot link"
			 << endl;
		return;
	}
	if (!robotExistsInWorld(robot_name, link_name)) {
		cout << "WARNING: robot [" << robot_name << "] link [" << link_name
			 << "] not found in graphics world, cannot attach a camera to it"
			 << endl;
		return;
	}
	if (_camera_link_attachments.find(camera_name) !=
		_camera_link_attachments.end()) {
		cout << "camera [" << camera_name
			 << "] already attached to a robot or object. detach first" << endl;
		return;
	}
	_camera_link_attachments[camera_name] =
		std::make_shared<CameraLinkAttachment>(robot_name, link_name,
											   pose_in_link);
}

void SaiGraphics::attachCameraToObject(const std::string& camera_name,
										const std::string& object_name,
										const Eigen::Affine3d& pose_in_object) {
	if (!cameraExistsInWorld(camera_name)) {
		cout << "WARNING: camera [" << camera_name
			 << "] not found in graphics world, cannot attach to object"
			 << endl;
		return;
	}
	if (!dynamicObjectExistsInWorld(object_name) &&
		!staticObjectExistsInWorld(object_name)) {
		cout << "WARNING: object [" << object_name
			 << "] object not found in graphics world, cannot attach a camera "
				"to it"
			 << endl;
		return;
	}
	if (_camera_link_attachments.find(camera_name) !=
		_camera_link_attachments.end()) {
		cout << "camera [" << camera_name
			 << "] already attached to a robot or object. detach first" << endl;
		return;
	}
	_camera_link_attachments[camera_name] =
		std::make_shared<CameraLinkAttachment>(object_name, "", pose_in_object);
}

void SaiGraphics::detachCameraFromRobotOrObject(
	const std::string& camera_name) {
	if (!cameraExistsInWorld(camera_name)) {
		cout << "WARNING: camera [" << camera_name
			 << "] not found in graphics world, cannot detach from robot or "
				"object"
			 << endl;
		return;
	}
	if (_camera_link_attachments.find(camera_name) ==
		_camera_link_attachments.end()) {
		cout << "camera [" << camera_name
			 << "] not attached to any object or robot" << endl;
		return;
	}
	_camera_link_attachments.erase(camera_name);
}

void SaiGraphics::addForceSensorDisplay(
	const SaiModel::ForceSensorData& sensor_data) {
	if (findForceSensorDisplay(sensor_data.robot_or_object_name,
							   sensor_data.link_name) != -1) {
		std::cout << "\n\nWARNING: only one force sensor is supported per "
					 "link in SaiGraphics::addForceSensorDisplay. Not "
					 "adding the second one\n"
				  << std::endl;
		return;
	}
	if (robotExistsInWorld(sensor_data.robot_or_object_name,
						   sensor_data.link_name)) {
		_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
			sensor_data.robot_or_object_name, sensor_data.link_name,
			sensor_data.transform_in_link,
			_robot_models.at(sensor_data.robot_or_object_name), _world));
	} else if (dynamicObjectExistsInWorld(sensor_data.robot_or_object_name)) {
		_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
			sensor_data.robot_or_object_name, sensor_data.link_name,
			sensor_data.transform_in_link,
			_dyn_objects_pose.at(sensor_data.robot_or_object_name), _world));
	} else if (staticObjectExistsInWorld(sensor_data.robot_or_object_name)) {
		_force_sensor_displays.push_back(std::make_shared<ForceSensorDisplay>(
			sensor_data.robot_or_object_name, sensor_data.link_name,
			sensor_data.transform_in_link,
			_static_objects_pose.at(sensor_data.robot_or_object_name), _world));
	} else {
		std::cout << "\n\nWARNING: trying to add a force sensor display to an "
					 "unexisting robot or link in "
					 "SaiGraphics::addForceSensorDisplay\n"
				  << std::endl;
	}
}

void SaiGraphics::updateDisplayedForceSensor(
	const SaiModel::ForceSensorData& force_data) {
	int sensor_index = findForceSensorDisplay(force_data.robot_or_object_name,
											  force_data.link_name);
	if (sensor_index == -1) {
		throw std::invalid_argument(
			"no force sensor on robot " + force_data.robot_or_object_name +
			" on link " + force_data.link_name +
			". Impossible to update the displayed force in graphics world");
		return;
	}
	if (!_force_sensor_displays.at(sensor_index)
			 ->T_link_sensor()
			 .isApprox(force_data.transform_in_link)) {
		throw std::invalid_argument(
			"transformation matrix between link and sensor inconsistent "
			"between the input force_data and the sensor_display in "
			"SaiGraphics::updateDisplayedForceSensor");
		return;
	}
	_force_sensor_displays.at(sensor_index)
		->update(force_data.force_world_frame, force_data.moment_world_frame);
}

void SaiGraphics::addMuscleTendonPathDisplay(
	const std::string& muscle_xml_path, const std::string& robot_name,
	const double line_width) {
	auto robot_it = _robot_models.find(robot_name);
	if (robot_it == _robot_models.end()) {
		throw std::invalid_argument(
			"robot not found in SaiGraphics::addMuscleTendonPathDisplay");
	}
	const auto& robot_model = robot_it->second;
	const auto muscle_system = SaiModel::parseMuscleXML(
		muscle_xml_path, [&robot_model](const std::string& link_name) {
			return robot_model->isLinkInRobot(link_name);
		});

	for (const auto& muscle : muscle_system.muscles) {
		const auto& waypoints = muscle.contractor.muscle_tendon_path;
		const auto limb_color =
			muscleLimbGroupColor(classifyMuscleLimbGroup(muscle));
		for (size_t i = 0; i < waypoints.size(); ++i) {
			const Eigen::Vector3d point = robot_model->positionInWorld(
				waypoints[i].link_name, waypoints[i].point);
			auto sphere_color =
				(i == 0 || i + 1 == waypoints.size())
					? muscleEndpointWaypointColor()
					: muscleIntermediateWaypointColor();

			auto* waypoint_sphere = new chai3d::cShapeSphere(0.004);
			waypoint_sphere->setLocalPos(chai3d::cVector3d(point));
			waypoint_sphere->m_material->setColor(sphere_color);

			_world->addChild(waypoint_sphere);
			_muscle_tendon_waypoints.push_back(
				{robot_name, muscle.muscle_name, waypoints[i], waypoint_sphere});
		}

		for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
			const Eigen::Vector3d point_a =
				robot_model->positionInWorld(waypoints[i].link_name,
											 waypoints[i].point);
			const Eigen::Vector3d point_b =
				robot_model->positionInWorld(waypoints[i + 1].link_name,
											 waypoints[i + 1].point);
			auto* display_line = new chai3d::cShapeLine();
			display_line->m_pointA = chai3d::cVector3d(point_a);
			display_line->m_pointB = chai3d::cVector3d(point_b);
			display_line->m_colorPointA = limb_color;
			display_line->m_colorPointB = limb_color;
			display_line->setLineWidth(line_width);

			_world->addChild(display_line);
			_muscle_tendon_path_lines.push_back(
				{robot_name, muscle.muscle_name, waypoints[i], waypoints[i + 1],
				 limb_color, line_width, display_line});
		}
	}
}

void SaiGraphics::clearMuscleTendonPathDisplay() {
	for (const auto& segment : _muscle_tendon_path_lines) {
		if (segment.line != nullptr) {
			_world->removeChild(segment.line);
			delete segment.line;
		}
	}
	_muscle_tendon_path_lines.clear();

	for (const auto& waypoint_display : _muscle_tendon_waypoints) {
		if (waypoint_display.sphere != nullptr) {
			_world->removeChild(waypoint_display.sphere);
			delete waypoint_display.sphere;
		}
	}
	_muscle_tendon_waypoints.clear();
}

void SaiGraphics::reloadMuscleTendonPathDisplay(
	const std::string& muscle_xml_path, const std::string& robot_name,
	const double line_width) {
	auto robot_it = _robot_models.find(robot_name);
	if (robot_it == _robot_models.end()) {
		throw std::invalid_argument(
			"robot not found in SaiGraphics::reloadMuscleTendonPathDisplay");
	}
	const auto& robot_model = robot_it->second;
	const auto muscle_system = SaiModel::parseMuscleXML(
		muscle_xml_path, [&robot_model](const std::string& link_name) {
			return robot_model->isLinkInRobot(link_name);
		});

	clearMuscleTendonPathDisplay();

	for (const auto& muscle : muscle_system.muscles) {
		const auto& waypoints = muscle.contractor.muscle_tendon_path;
		const auto limb_color =
			muscleLimbGroupColor(classifyMuscleLimbGroup(muscle));
		for (size_t i = 0; i < waypoints.size(); ++i) {
			const Eigen::Vector3d point = robot_model->positionInWorld(
				waypoints[i].link_name, waypoints[i].point);
			auto sphere_color =
				(i == 0 || i + 1 == waypoints.size())
					? muscleEndpointWaypointColor()
					: muscleIntermediateWaypointColor();

			auto* waypoint_sphere = new chai3d::cShapeSphere(0.004);
			waypoint_sphere->setLocalPos(chai3d::cVector3d(point));
			waypoint_sphere->m_material->setColor(sphere_color);

			_world->addChild(waypoint_sphere);
			_muscle_tendon_waypoints.push_back(
				{robot_name, muscle.muscle_name, waypoints[i], waypoint_sphere});
		}

		for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
			const Eigen::Vector3d point_a =
				robot_model->positionInWorld(waypoints[i].link_name,
											 waypoints[i].point);
			const Eigen::Vector3d point_b =
				robot_model->positionInWorld(waypoints[i + 1].link_name,
											 waypoints[i + 1].point);
			auto* display_line = new chai3d::cShapeLine();
			display_line->m_pointA = chai3d::cVector3d(point_a);
			display_line->m_pointB = chai3d::cVector3d(point_b);
			display_line->m_colorPointA = limb_color;
			display_line->m_colorPointB = limb_color;
			display_line->setLineWidth(line_width);

			_world->addChild(display_line);
			_muscle_tendon_path_lines.push_back(
				{robot_name, muscle.muscle_name, waypoints[i], waypoints[i + 1],
				 limb_color, line_width, display_line});
		}
	}
}

void SaiGraphics::updateMuscleTendonPathDisplay() {
	for (const auto& segment : _muscle_tendon_path_lines) {
		auto robot_it = _robot_models.find(segment.robot_name);
		if (robot_it == _robot_models.end()) {
			throw std::invalid_argument(
				"robot not found in SaiGraphics::updateMuscleTendonPathDisplay");
		}

		const auto& robot_model = robot_it->second;
		const Eigen::Vector3d point_a = robot_model->positionInWorld(
			segment.point_a_waypoint.link_name, segment.point_a_waypoint.point);
		const Eigen::Vector3d point_b = robot_model->positionInWorld(
			segment.point_b_waypoint.link_name, segment.point_b_waypoint.point);

		segment.line->m_pointA = chai3d::cVector3d(point_a);
		segment.line->m_pointB = chai3d::cVector3d(point_b);
	}

	for (const auto& waypoint_display : _muscle_tendon_waypoints) {
		auto robot_it = _robot_models.find(waypoint_display.robot_name);
		if (robot_it == _robot_models.end()) {
			throw std::invalid_argument(
				"robot not found in SaiGraphics::updateMuscleTendonPathDisplay");
		}

		const auto& robot_model = robot_it->second;
		const Eigen::Vector3d point = robot_model->positionInWorld(
			waypoint_display.waypoint.link_name, waypoint_display.waypoint.point);
		waypoint_display.sphere->setLocalPos(chai3d::cVector3d(point));
	}
}

bool SaiGraphics::robotExistsInWorld(const std::string& robot_name,
									  const std::string& link_name) const {
	auto it = _robot_models.find(robot_name);
	if (it == _robot_models.end()) {
		return false;
	}
	if (link_name != "") {
		return _robot_models.at(robot_name)->isLinkInRobot(link_name);
	}
	return true;
}

bool SaiGraphics::dynamicObjectExistsInWorld(
	const std::string& object_name) const {
	auto it = _dyn_objects_pose.find(object_name);
	if (it == _dyn_objects_pose.end()) {
		return false;
	}
	return true;
}

bool SaiGraphics::staticObjectExistsInWorld(
	const std::string& object_name) const {
	auto it = _static_objects_pose.find(object_name);
	if (it == _static_objects_pose.end()) {
		return false;
	}
	return true;
}

bool SaiGraphics::cameraExistsInWorld(const std::string& camera_name) const {
	for (const std::string name : _camera_names) {
		if (name == camera_name) {
			return true;
		}
	}
	return false;
}

int SaiGraphics::findForceSensorDisplay(
	const std::string& robot_or_object_name,
	const std::string& link_name) const {
	for (int i = 0; i < _force_sensor_displays.size(); ++i) {
		if (_force_sensor_displays.at(i)->robot_or_object_name() ==
				robot_or_object_name &&
			_force_sensor_displays.at(i)->link_name() == link_name) {
			return i;
		}
	}
	return -1;
}

void SaiGraphics::addUIForceInteraction(
	const std::string& robot_or_object_name,
	const bool interact_at_object_center) {
	bool is_robot = robotExistsInWorld(robot_or_object_name);
	bool is_object = dynamicObjectExistsInWorld(robot_or_object_name);
	if (!is_robot && !is_object) {
		throw std::invalid_argument(
			"robot or dynamic object not found in "
			"SaiGraphics::addUIForceInteraction");
	}
	for (auto widget : _ui_force_widgets) {
		if (robot_or_object_name == widget->getRobotOrObjectName()) {
			return;
		}
	}
	chai3d::cShapeLine* display_line = new chai3d::cShapeLine();
	_world->addChild(display_line);
	if (is_robot) {
		_ui_force_widgets.push_back(std::make_shared<UIForceWidget>(
			robot_or_object_name, interact_at_object_center,
			_robot_models[robot_or_object_name], display_line));
	} else {
		_ui_force_widgets.push_back(std::make_shared<UIForceWidget>(
			robot_or_object_name, interact_at_object_center,
			_dyn_objects_pose[robot_or_object_name],
			_object_velocities[robot_or_object_name], display_line));
	}
}

Eigen::VectorXd SaiGraphics::getUITorques(
	const std::string& robot_or_object_name) {
	bool is_robot = robotExistsInWorld(robot_or_object_name);
	bool is_object = dynamicObjectExistsInWorld(robot_or_object_name);
	if (!is_robot && !is_object) {
		throw std::invalid_argument(
			"robot or dynamic object not found in SaiGraphics::getUITorques");
	}
	for (auto widget : _ui_force_widgets) {
		if (robot_or_object_name == widget->getRobotOrObjectName()) {
			return widget->getUIJointTorques();
		}
	}
	return is_robot ? Eigen::VectorXd::Zero(
						  _robot_models[robot_or_object_name]->dof())
					: Eigen::VectorXd::Zero(6);
}

const std::vector<std::string> SaiGraphics::getRobotNames() const {
	std::vector<std::string> robot_names;
	for (const auto& it : _robot_filenames) {
		robot_names.push_back(it.first);
	}
	return robot_names;
}

const std::vector<std::string> SaiGraphics::getObjectNames() const {
	std::vector<std::string> object_names;
	for (const auto& it : _dyn_objects_pose) {
		object_names.push_back(it.first);
	}
	return object_names;
}

void SaiGraphics::renderBlackScreen() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);
	glfwSwapBuffers(_window);
	glfwPollEvents();
}

cImagePtr SaiGraphics::getCameraImage(const std::string& camera_name,
									   const int width, const int height) {
	if (!cameraExistsInWorld(camera_name)) {
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

void SaiGraphics::renderGraphicsWorld() {
	// swap camera if needed
	if (consume_first_press(NEXT_CAMERA_KEY)) {
		_current_camera_index =
			(_current_camera_index + 1) % _camera_names.size();
	}
	if (consume_first_press(PREV_CAMERA_KEY)) {
		_current_camera_index =
			(_current_camera_index - 1) % _camera_names.size();
	}
	const std::string camera_name = _camera_names[_current_camera_index];

	// update graphics. this automatically waits for the correct amount of time
	glfwGetFramebufferSize(_window, &_window_width, &_window_height);
	glfwSwapBuffers(_window);
	glFinish();

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
	int wwidth_scr, wheight_scr;
	glfwGetWindowSize(_window, &wwidth_scr, &wheight_scr);

	// 0 - scroll event
	double scroll_value = 0.0;
	if (!mouse_scroll_buffer.empty()) {
		scroll_value = mouse_scroll_buffer.front();
		mouse_scroll_buffer.pop_front();
	}

	const bool joint_slider_consumed_input = updateJointSliderDropdown(
		camera_name, cursorx, cursory, wwidth_scr, wheight_scr, scroll_value);
	if (joint_slider_consumed_input) {
		scroll_value = 0.0;
	}

	// 1 - mouse right button to generate a force/torque
	if (!_joint_slider_interaction_occurring && is_pressed(GLFW_MOUSE_BUTTON_RIGHT)) {
		if (consume_first_press(GLFW_MOUSE_BUTTON_RIGHT)) {
			for (auto widget : _ui_force_widgets) {
				widget->setEnable(true);
			}
		}

		int viewx = floor(cursorx / wwidth_scr * _window_width);
		int viewy = floor(cursory / wheight_scr * _window_height);

		for (auto widget : _ui_force_widgets) {
			if (widget->getState() == UIForceWidget::Active) {
				_right_click_interaction_occurring = true;
			}

			double depth_change = 0.0;
			if (_right_click_interaction_occurring) {
				depth_change = 0.1 * scroll_value;
				if (is_pressed(ZOOM_IN_KEY)) {
					depth_change += 0.05;
				}
				if (is_pressed(ZOOM_OUT_KEY)) {
					depth_change -= 0.05;
				}
			}
			if (is_pressed(GLFW_KEY_LEFT_SHIFT)) {
				widget->setMomentMode();
			} else {
				widget->setForceMode();
			}
			widget->setInteractionParams(getCamera(camera_name), viewx,
										 _window_height - viewy, _window_width,
										 _window_height, depth_change);
		}
	} else {
		for (auto widget : _ui_force_widgets) {
			widget->setEnable(false);
		}
		_right_click_interaction_occurring = false;
	}

	// 2 - mouse left button for camera motion
	if (!_right_click_interaction_occurring && !joint_slider_consumed_input) {
		if (is_pressed(GLFW_MOUSE_BUTTON_LEFT)) {
			if (is_pressed(GLFW_KEY_LEFT_CONTROL)) {
				Eigen::Vector3d cam_motion =
					0.01 * (mouse_x_increment * cam_right_axis -
							mouse_y_increment * camera_up_axis);
				camera_pos -= cam_motion;
				camera_lookat_point -= cam_motion;
			} else if (is_pressed(GLFW_KEY_LEFT_ALT) ||
					   is_pressed(GLFW_KEY_LEFT_SHIFT)) {
				Eigen::Vector3d cam_motion =
					0.02 * mouse_y_increment * cam_depth_axis;
				camera_pos -= cam_motion;
				camera_lookat_point -= cam_motion;
			} else {
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
		if (is_pressed(GLFW_MOUSE_BUTTON_MIDDLE)) {
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
		if (is_pressed(CAMERA_RIGHT_KEY)) {
			camera_pos += 0.05 * cam_right_axis;
			camera_lookat_point += 0.05 * cam_right_axis;
		}
		if (is_pressed(CAMERA_LEFT_KEY)) {
			camera_pos -= 0.05 * cam_right_axis;
			camera_lookat_point -= 0.05 * cam_right_axis;
		}
		if (is_pressed(CAMERA_UP_KEY)) {
			camera_pos += 0.05 * camera_up_axis;
			camera_lookat_point += 0.05 * camera_up_axis;
		}
		if (is_pressed(CAMERA_DOWN_KEY)) {
			camera_pos -= 0.05 * camera_up_axis;
			camera_lookat_point -= 0.05 * camera_up_axis;
		}
		if (is_pressed(ZOOM_IN_KEY)) {
			camera_pos += 0.1 * cam_depth_axis;
			camera_lookat_point += 0.1 * cam_depth_axis;
		}
		if (is_pressed(ZOOM_OUT_KEY)) {
			camera_pos -= 0.1 * cam_depth_axis;
			camera_lookat_point -= 0.1 * cam_depth_axis;
		}

		if (consume_first_press(SHOW_CAMERA_POS_KEY)) {
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
		_camera_link_attachments.end()) {
		const auto attachment = _camera_link_attachments.at(camera_name);

		Affine3d camera_pose;
		if (attachment->link_name == "") {
			camera_pose = getObjectPose(attachment->model_name) *
						  attachment->pose_in_link;
		} else {
			camera_pose =
				_robot_models.at(attachment->model_name)
					->transformInWorld(attachment->link_name,
									   attachment->pose_in_link.translation(),
									   attachment->pose_in_link.rotation());
		}
		setCameraPose(camera_name, camera_pose);
	}

	applyJointSliderOverrides();

	// update shadow maps
	_world->updateShadowMaps();
	updateMuscleTendonHoverLabel(camera_name, cursorx, cursory, wwidth_scr,
								   wheight_scr);
	updateJointFrameHoverLabel(camera_name, cursorx, cursory, wwidth_scr,
								 wheight_scr);

	render(camera_name);
}

static void updateGraphicsLink(
	cRobotLink* link, std::shared_ptr<SaiModel::SaiModel> robot_model) {
	cVector3d local_pos;
	cMatrix3d local_rot;

	// get link name
	std::string link_name = link->m_name;

	// get chai parent name
	cGenericObject* parent = link->getParent();
	std::string parent_name = parent->m_name;

	// if parent is cRobotBase, then simply get transform relative to base from
	// model
	if (dynamic_cast<cRobotBase*>(parent) != NULL) {
		Eigen::Affine3d T = robot_model->transform(link_name);
		local_pos = cVector3d(T.translation());
		local_rot = cMatrix3d(T.rotation());
	} else if (dynamic_cast<cRobotLink*>(parent) != NULL) {
		// if parent is cRobotLink, then calculate transform for both links,
		// then apply inverse transform
		Eigen::Affine3d T_me, T_parent, T_rel;
		T_me = robot_model->transform(link_name);
		T_parent = robot_model->transform(parent_name);
		T_rel = T_parent.inverse() * T_me;
		local_pos = cVector3d(T_rel.translation());
		local_rot = cMatrix3d(T_rel.rotation());
	} else {
		cerr << "Parent to link " << link_name << " is neither link nor base"
			 << endl;
		abort();
		// TODO: throw exception
	}
	link->setLocalPos(local_pos);
	link->setLocalRot(local_rot);

	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < link->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(link->getChild(i));
		if (child != NULL) {
			updateGraphicsLink(child, robot_model);
		}
	}
}

// update frame for a particular robot
void SaiGraphics::updateRobotGraphics(const std::string& robot_name,
									   const Eigen::VectorXd& joint_angles) {
	auto it = _robot_models.find(robot_name);
	if (it == _robot_models.end()) {
		throw std::invalid_argument(
			"Robot not found in SaiGraphics::updateRobotGraphics");
	}
	updateRobotGraphics(
		robot_name, joint_angles,
		Eigen::VectorXd::Zero(_robot_models.at(robot_name)->dof()));
}

void SaiGraphics::updateRobotGraphics(
	const std::string& robot_name, const Eigen::VectorXd& joint_angles,
	const Eigen::VectorXd& joint_velocities) {
	// update corresponfing robot model
	auto it = _robot_models.find(robot_name);
	if (it == _robot_models.end()) {
		throw std::invalid_argument(
			"Robot not found in SaiGraphics::updateRobotGraphics");
	}
	auto robot_model = _robot_models.at(robot_name);
	if (joint_angles.size() != robot_model->qSize()) {
		throw std::invalid_argument(
			"size of joint angles inconsistent with robot model in "
			"SaiGraphics::updateRobotGraphics");
	}
	if (joint_velocities.size() != robot_model->dof()) {
		throw std::invalid_argument(
			"size of joint velocities inconsistent with robot model in "
			"SaiGraphics::updateRobotGraphics");
	}
	robot_model->setQ(joint_angles);
	robot_model->setDq(joint_velocities);
	robot_model->updateKinematics();

	// get robot base object in chai world
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		// TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}
	// recursively update graphics for all children
	cRobotLink* link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (link != NULL) {
			updateGraphicsLink(link, robot_model);
		}
	}
}

void SaiGraphics::updateObjectGraphics(
	const std::string& object_name, const Eigen::Affine3d& object_pose,
	const Eigen::Vector6d& object_velocity) {
	if (!dynamicObjectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"dynamic object not found in SaiGraphics::updateObjectGraphics");
	}
	cGenericObject* object = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (object_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			object = _world->getChild(i);
			if (object != NULL) {
				break;
			}
		}
	}
	if (object == NULL) {
		// TODO: throw exception
		cerr << "Could not find object in chai world: " << object_name << endl;
		abort();
	}

	// update pose
	*_dyn_objects_pose.at(object_name) = object_pose;
	*_object_velocities.at(object_name) = object_velocity;
	object->setLocalPos(object_pose.translation());
	object->setLocalRot(object_pose.rotation());
}

Eigen::VectorXd SaiGraphics::getRobotJointPos(const std::string& robot_name) {
	auto it = _robot_models.find(robot_name);
	if (it == _robot_models.end()) {
		throw std::invalid_argument(
			"robot not found in SaiGraphics::getRobotJointPos");
	}
	return _robot_models[robot_name]->q();
}

Eigen::Affine3d SaiGraphics::getObjectPose(const std::string& object_name) {
	if (!dynamicObjectExistsInWorld(object_name)) {
		throw std::invalid_argument(
			"dynamic object not found in SaiGraphics::getObjectPose");
	}
	return *_dyn_objects_pose.at(object_name);
}

void SaiGraphics::render(const std::string& camera_name) {
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
void SaiGraphics::getCameraPoseInternal(const std::string& camera_name,
										 Eigen::Vector3d& ret_position,
										 Eigen::Vector3d& ret_vertical_axis,
										 Eigen::Vector3d& ret_lookat_point) {
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
void SaiGraphics::setCameraPoseInternal(const std::string& camera_name,
										 const Eigen::Vector3d& position,
										 const Eigen::Vector3d& vertical_axis,
										 const Eigen::Vector3d& lookat_point) {
	auto camera = getCamera(camera_name);
	cVector3d pos(position[0], position[1], position[2]);
	cVector3d vert(vertical_axis[0], vertical_axis[1], vertical_axis[2]);
	cVector3d look(lookat_point[0], lookat_point[1], lookat_point[2]);
	camera->set(pos, look, vert);
}

// get camera object
cCamera* SaiGraphics::getCamera(const std::string& camera_name) {
	if (!cameraExistsInWorld(camera_name)) {
		throw std::invalid_argument(
			"camera not found in SaiGraphics::getCamera");
	}
	return _camera_frame_buffers.at(camera_name)->getCamera();
}

cRobotLink* SaiGraphics::findLinkObjectInParentLinkRecursive(
	cRobotLink* parent, const std::string& link_name) {
	// call on children
	cRobotLink* child;
	cRobotLink* ret_link = NULL;
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(parent->getChild(i));
		if (child != NULL) {
			if (child->m_name == link_name) {
				ret_link = child;
				break;
			} else {
				ret_link =
					findLinkObjectInParentLinkRecursive(child, link_name);
				if (ret_link != NULL) {
					break;
				}
			}
		}
	}
	return ret_link;
}

cRobotLink* SaiGraphics::findLink(const std::string& robot_name,
								   const std::string& link_name) {
	// get robot base
	cRobotBase* base = NULL;
	for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
		if (robot_name == _world->getChild(i)->m_name) {
			// cast to cRobotBase
			base = dynamic_cast<cRobotBase*>(_world->getChild(i));
			if (base != NULL) {
				break;
			}
		}
	}
	if (base == NULL) {
		// TODO: throw exception
		cerr << "Could not find robot in chai world: " << robot_name << endl;
		abort();
	}

	cRobotLink* target_link = NULL;
	// get target link
	cRobotLink* base_link;
	for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
		base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
		if (base_link != NULL) {
			if (base_link->m_name == link_name) {
				target_link = base_link;
				break;
			} else {
				target_link =
					findLinkObjectInParentLinkRecursive(base_link, link_name);
				if (target_link != NULL) {
					break;
				}
			}
		}
	}
	return target_link;
}

const cRobotLink* SaiGraphics::findParentRobotLink(
	const cGenericObject* object) const {
	while (object != nullptr) {
		auto* link = dynamic_cast<const cRobotLink*>(object);
		if (link != nullptr) {
			return link;
		}
		object = object->getParent();
	}
	return nullptr;
}

void SaiGraphics::showLinkFrameRecursive(cRobotLink* parent, bool show_frame,
										  const double frame_pointer_length) {
	// call on children
	cRobotLink* child;
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		child = dynamic_cast<cRobotLink*>(parent->getChild(i));
		if (child != NULL) {
			child->setFrameSize(frame_pointer_length, false);
			child->setShowFrame(show_frame, false);
			showLinkFrameRecursive(child, show_frame, frame_pointer_length);
		}
	}
}

void SaiGraphics::showLinkFrame(bool show_frame,
								 const std::string& robot_or_object_name,
								 const std::string& link_name,
								 const double frame_pointer_length) {
	if (link_name.empty()) {  // apply to all links
		cGenericObject* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_or_object_name == _world->getChild(i)->m_name) {
				base = _world->getChild(i);
				if (base != NULL) {
					break;
				}
			}
		}
		if (base == NULL) {
			cerr << "Could not find robot in chai world: "
				 << robot_or_object_name << ". Cannot show frame." << endl;
		}
		base->setFrameSize(frame_pointer_length, false);
		base->setShowFrame(show_frame, false);
		// get target link
		cRobotLink* base_link;
		for (unsigned int i = 0; i < base->getNumChildren(); ++i) {
			base_link = dynamic_cast<cRobotLink*>(base->getChild(i));
			if (base_link != NULL) {
				base_link->setFrameSize(frame_pointer_length, false);
				base_link->setShowFrame(show_frame, false);
				showLinkFrameRecursive(base_link, show_frame,
									   frame_pointer_length);
			}
		}
	} else {
		auto target_link = findLink(robot_or_object_name, link_name);
		target_link->setFrameSize(frame_pointer_length, false);
		target_link->setShowFrame(show_frame, false);
	}
}

void SaiGraphics::showMovableJointFrames(
	bool show_frame, const std::string& robot_name,
	const double frame_pointer_length, const bool show_joint_name_on_hover) {
	const auto robot_it = _robot_models.find(robot_name);
	if (robot_it == _robot_models.end()) {
		cerr << "Could not find robot model in graphics world: " << robot_name
			 << ". Cannot show movable joint frames." << endl;
		abort();
	}

	_joint_frame_displays.erase(
		std::remove_if(_joint_frame_displays.begin(), _joint_frame_displays.end(),
					   [&](const JointFrameDisplay& joint_frame_display) {
						   return joint_frame_display.robot_name == robot_name;
					   }),
		_joint_frame_displays.end());

	std::unordered_set<std::string> visited_child_links;
	for (const auto& joint_name : robot_it->second->jointNames()) {
		const auto child_link_name = robot_it->second->childLinkName(joint_name);
		if (child_link_name.empty() ||
			!visited_child_links.insert(child_link_name).second) {
			continue;
		}

		auto* target_link = findLink(robot_name, child_link_name);
		if (target_link == NULL) {
			cerr << "Could not find child link '" << child_link_name
				 << "' for movable joint '" << joint_name << "' on robot '"
				 << robot_name << "'. Skipping frame display for this joint."
				 << endl;
			continue;
		}
		target_link->setFrameSize(frame_pointer_length, false);
		target_link->setShowFrame(show_frame, false);
		if (show_frame && show_joint_name_on_hover) {
			_joint_frame_displays.push_back(
				{robot_name, joint_name, target_link, frame_pointer_length});
		}
	}
}

void SaiGraphics::showWireMesh(bool show_wiremesh,
								const std::string& robot_or_object_name,
								const std::string& link_name) {
	if (link_name.empty()) {  // apply to all links
		cGenericObject* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_or_object_name == _world->getChild(i)->m_name) {
				base = _world->getChild(i);
				if (base != NULL) {
					break;
				}
			}
		}
		if (base == NULL) {
			cerr << "Could not find robot or object in chai graphics world: "
				 << robot_or_object_name << ". Cannot show wire mesh." << endl;
		}
		base->setWireMode(show_wiremesh, true);
	} else {
		auto target_link = findLink(robot_or_object_name, link_name);
		target_link->setWireMode(show_wiremesh, true);
	}
}

void SaiGraphics::setRenderingEnabled(const bool rendering_enabled,
									   const string robot_or_object_name,
									   const string link_name) {
	if (link_name.empty()) {  // apply to all links
		cGenericObject* base = NULL;
		for (unsigned int i = 0; i < _world->getNumChildren(); ++i) {
			if (robot_or_object_name == _world->getChild(i)->m_name) {
				base = _world->getChild(i);
				if (base != NULL) {
					break;
				}
			}
		}
		if (base == NULL) {
			cerr << "Could not find robot or object in chai graphics world: "
				 << robot_or_object_name << ". Cannot enable/disable rendering."
				 << endl;
		}
		base->setEnabled(rendering_enabled, true);
	} else {
		auto target_link = findLink(robot_or_object_name, link_name);
		target_link->setEnabled(rendering_enabled, false);
		cGenericObject* child;
		for (unsigned int i = 0; i < target_link->getNumChildren(); ++i) {
			child = target_link->getChild(i);
			// only apply to children that are visual elements (supposed to have
			// the same name), not children links (which will have different
			// names)
			if (child->m_name == link_name) {
				child->setEnabled(rendering_enabled, false);
			}
		}
	}
}

}  // namespace SaiGraphics
