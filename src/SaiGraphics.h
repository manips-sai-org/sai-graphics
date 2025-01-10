/**
 * \file SaiGraphics.h
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#ifndef SAI_GRAPHICS_H
#define SAI_GRAPHICS_H

#include <chai3d.h>

#include "SaiModel.h"
#include "widgets/ForceSensorDisplay.h"
#include "widgets/UIForceWidget.h"

// clang-format off
#include <GLFW/glfw3.h>	 //must be loaded after loading opengl/glew
// clang-format on

namespace SaiGraphics {

/**
 * @brief Structure to store the attachment of a camera to a link or object
 *
 */
struct CameraLinkAttachment {
	std::string model_name;
	std::string link_name;	// empty if object
	Eigen::Affine3d pose_in_link;

	CameraLinkAttachment(const std::string& model_name,
						 const std::string& link_name,
						 const Eigen::Affine3d& pose_in_link)
		: model_name(model_name),
		  link_name(link_name),
		  pose_in_link(pose_in_link) {}
};

/**
 * @brief Class that represents a visual model of the virtual world.
 *
 */
class SaiGraphics {
public:
	/**
	 * @brief Creates a Chai graphics interface object that contains a visual
	 * model of the virtual world.
	 * @param path_to_world_file A path to the file containing the model of the
	 * virtual world (urdf and yml files supported).
	 * @param verbose To display information about the robot model creation in
	 * the terminal or not.
	 */
	SaiGraphics(const std::string& path_to_world_file,
				const std::string& window_name = "sai world",
				bool verbose = false);

	/**
	 * @brief Destructor
	 *
	 */
	~SaiGraphics();

	/**
	 * @brief resets the rendered world and re initializes it with the new world
	 * file
	 *
	 * @param path_to_world_file world file to render
	 * @param verbose print info to terminal or not
	 */
	void resetWorld(const std::string& path_to_world_file,
					const bool verbose = false);

	/**
	 * @brief returns true is the window is open and should stay open
	 */
	bool isWindowOpen() { return !glfwWindowShouldClose(_window); }

	/**
	 * @brief Call this function to render a black screen in the window
	 *
	 */
	void renderBlackScreen();

	/**
	 * @brief renders the graphics world from the current camera (needs to be
	 * called after all the update functions i.e. updateRobotGraphics)
	 */
	void renderGraphicsWorld();

	/**
	 * @brief Gets the camera image for any camera in the world (not necessarily
	 * the current one)
	 *
	 * @param camera_name name of the camera
	 * @param width width of the image in pixels
	 * @param height height of the image in pixels
	 * @return chai3d::cImagePtr image from the camera as a chai3d image
	 */
	chai3d::cImagePtr getCameraImage(const std::string& camera_name,
									 const int width = 720,
									 const int height = 480);

	/**
	 * @brief remove all interactions widgets
	 * after calling that function, right clicking on the window won't
	 * display lines and generate forces/joint torques
	 *
	 */
	void clearUIForceWidgets() { _ui_force_widgets.clear(); }

	/**
	 * @brief get the joint torques from the ui interaction (right click on
	 * robot link) for a given robot
	 *
	 * @param robot_name name of the robot for which we want the joint torques
	 * @return joint torques from UI interaction for that robot
	 */
	Eigen::VectorXd getUITorques(const std::string& robot_name);

	/**
	 * @brief Enable interacting with a specific robot by right clicking on the
	 * display window
	 *
	 * @param robot_name name of the robot
	 * @param robot_model model of the robot
	 */
	void addUIForceInteraction(const std::string& robot_name,
							   const bool interact_at_object_center = false);

	/**
	 * @brief Get the the names of the robots in the graphics world
	 *
	 * @return vector of robot names
	 */
	const std::vector<std::string> getRobotNames() const;

	/**
	 * @brief Get the the names of the objects in the graphics world
	 *
	 * @return vector of object names
	 */
	const std::vector<std::string> getObjectNames() const;

	/**
	 * @brief Update the graphics model for a robot in the virtual world.
	 * Provide the velocities if you want to use the ui interaction widget
	 * (because they are needed to compute the damping).
	 * @param robot_name Name of the robot for which model update is considered.
	 * @param joint_angles joint angles for that robot
	 * @param joint_velocities joint velocities for that robot
	 */
	void updateRobotGraphics(const std::string& robot_name,
							 const Eigen::VectorXd& joint_angles,
							 const Eigen::VectorXd& joint_velocities);

	/**
	 * @brief Update the graphics model for a robot in the virtual world,
	 * without providing the velocities (if there is a ui interaction widget,
	 * the damping will be set to 0).
	 * @param robot_name Name of the robot for which model update is considered.
	 * @param joint_angles joint angles for that robot
	 */
	void updateRobotGraphics(const std::string& robot_name,
							 const Eigen::VectorXd& joint_angles);

	/**
	 * @brief Update the graphics model for an object in the virtual world.
	 * @param object_name Name of the object for which model update is
	 * considered.
	 * @param object_pose  pose of the object in the world
	 */
	void updateObjectGraphics(
		const std::string& object_name, const Eigen::Affine3d& object_pose,
		const Eigen::Vector6d& object_velocity = Eigen::Vector6d::Zero());

	/**
	 * @brief Get the Joint positions of a given robot in the graphics world
	 *
	 * @param robot_name Name of the robot
	 * @return Eigen::VectorXd Joint positions of the robot
	 */
	Eigen::VectorXd getRobotJointPos(const std::string& robot_name);

	/**
	 * @brief Get the pose of an object in the graphics world
	 *
	 * @param object_name Name of the object
	 * @return Eigen::Affine3d Pose of the object
	 */
	Eigen::Affine3d getObjectPose(const std::string& object_name);

	/**
	* @brief Show frame for a particular link or all links on a robot.
		 This also causes the link graphics as well as graphics for any
		 child link to be displayed as wire mesh to allow the frame to be
		 seen.
	* @param show_frame Flag whether should show frame or not.
	* @param robot_name Robot name.
	* @param robot_name Link name. If left blank, all link frames are shown.
	* @param frame_pointer_length Axis arrow length in meters.
	*/
	void showLinkFrame(bool show_frame, const std::string& robot_name,
					   const std::string& link_name = "",
					   const double frame_pointer_length = 0.20);

	/**
	 * @brief Render wire mesh for a particular link or all links on a robot.
	 * @param show_wiremesh Flag whether should show wire mesh or not.
	 * @param robot_name Robot name.
	 * @param robot_name Link name. If left blank, all link frames are shown.
	 */
	void showWireMesh(bool show_wiremesh, const std::string& robot_name,
					  const std::string& link_name = "");

	/**
	 * @brief Set the Background color of the world
	 *
	 * @param red red component of the color between 0 and 1
	 * @param green green component of the color between 0 and 1
	 * @param blue blue component of the color between 0 and 1
	 */
	void setBackgroundColor(const double red, const double green,
							const double blue) {
		_world->setBackgroundColor(red, green, blue);
	}

	/// @brief Returns the current camera name.
	std::string getCurrentCameraName() const {
		return _camera_names[_current_camera_index];
	}

	/**
	 * @brief Sets the pose of a camera. The translation part of the pose is the
	 * position of the camera, and the rotation part is the orientation of the
	 * camera with the convention that the Z axis is the camera depth axis and
	 * the X axis is the right axis in the image (and therefore the image up is
	 * -Y).
	 *
	 * @param camera_name
	 * @param camera_pose
	 */
	void setCameraPose(const std::string& camera_name,
					   const Eigen::Affine3d& camera_pose);

	/**
	 * @brief Returns the camera pose for a given camera.
	 *
	 * @param camera_name name of the camera
	 * @return Eigen::Affine3d pose of the camera with the convention X right, Y
	 * down, Z forward
	 */
	Eigen::Affine3d getCameraPose(const std::string& camera_name);

	/**
	 * @brief Attach a camera to a robot link such that the camera moves
	 * automatically with said link. The pose of the camera cannot be set via
	 * the setCameraPose function (or via the usual ui controls in the
	 * visualization window) when it is attached to a robot link.
	 *
	 * @param camera_name name of the camera to attach
	 * @param robot_name name of the robot to attach the camera to
	 * @param link_name name of the link to attach the camera to
	 * @param pose_in_link pose of the camera in the link frame
	 */
	void attachCameraToRobotLink(const std::string& camera_name,
								 const std::string& robot_name,
								 const std::string& link_name,
								 const Eigen::Affine3d& pose_in_link);

	/**
	 * @brief Attach a camera to an object such that the camera moves
	 * automatically with said object. The pose of the camera cannot be set via
	 * the setCameraPose function (or via the usual ui controls in the
	 * visualization window) when it is attached to an object.
	 *
	 * @param camera_name name of the camera to attach
	 * @param object_name name of the object to attach the camera to
	 * @param pose_in_object pose of the camera in the object frame
	 */
	void attachCameraToObject(const std::string& camera_name,
							  const std::string& object_name,
							  const Eigen::Affine3d& pose_in_object);

	/// @brief Detach a camera from a robot or object.
	void detachCameraFromRobotOrObject(const std::string& camera_name);

	/**
	 * @brief adds a force sensor display to the graphics world. The force
	 * sensor data contains the name of robot or object, the link name and the
	 * pose of the sensor in the link frame. It should come from the simulation.
	 * Once attached, call the updateDisplayedForceSensor function to update the
	 * displayed force. A force will show as a green line and a moment as a red
	 * line.
	 *
	 * @param sensor_data force sensor data that contains the name of robot or
	 * object, the link name and the pose of the sensor in the link frame.
	 */
	void addForceSensorDisplay(const SaiModel::ForceSensorData& sensor_data);

	/**
	 * @brief updates the displayed force sensor with the new force and moment
	 * values. The force sensor data contains the informations about the robot
	 * or object, link name and pose of the sensor in the link frame to know
	 * which force sensor to update. It also contains the force and moment
	 * values to display.
	 *
	 * @param force_data force sensor data that contains all the necessary info
	 * and should come from the simulation.
	 */
	void updateDisplayedForceSensor(
		const SaiModel::ForceSensorData& force_data);

	/// @brief returns true if the given key is pressed, false otherwise
	bool isKeyPressed(int key) const {
		return glfwGetKey(_window, key) == GLFW_PRESS;
	}

	/**
	 * @brief Enable or disable rendering for a robot or object in the world. Of
	 * the rendering is disabled, the object will not be displayed in the
	 * visualizer window.
	 *
	 * @param rendering_enabled true to enable rendering, false to disable
	 * @param robot_or_object_name name of the robot or object
	 * @param link_name name of the link to render or not. Leave empty to apply
	 * to all links.
	 */
	void setRenderingEnabled(const bool rendering_enabled,
							 const string robot_or_object_name,
							 const string link_name = "");

	/// @brief returns true if the model exists in the world (robot or object),
	/// false otherwise
	bool modelExistsInWorld(const std::string& model_name) const {
		return robotExistsInWorld(model_name) ||
			   dynamicObjectExistsInWorld(model_name) ||
			   staticObjectExistsInWorld(model_name);
	}

	/// @brief returns true if the robot exists in the world, false otherwise
	bool robotExistsInWorld(const std::string& robot_name,
							const std::string& link_name = "") const;

	/// @brief returns true if the object exists in the world and is a dynamic
	/// object, false otherwise
	bool dynamicObjectExistsInWorld(const std::string& object_name) const;

	/// @brief returns true if the object exists in the world and is a static
	/// object, false otherwise
	bool staticObjectExistsInWorld(const std::string& object_name) const;

	/// @brief returns true if the camera exists in the world, false otherwise
	bool cameraExistsInWorld(const std::string& camera_name) const;

private:
	/**
	 * @brief Initialize the world with the given world file
	 *
	 * @param path_to_world_file path to the world file
	 * @param verbose print info to terminal or not
	 */
	void initializeWorld(const std::string& path_to_world_file,
						 const bool verbose);

	/**
	 * @brief clears the world and all the objects in it
	 *
	 */
	void clearWorld();

	/**
	 * @brief initialize the glfw window with the given window name
	 *
	 * @param window_name
	 */
	void initializeWindow(const std::string& window_name);

	/**
	 * @brief Render the virtual world to the current context.
	 * 	NOTE: the correct context should have been selected prior to this.
	 * @param camera_name Camera name to be rendered from.
	 * @param window_width Width of viewport in screen co-ordinates.
	 * @param window_height Height of viewport in screen co-ordinates.
	 * @param display_context_id ID for the context to display in. This ID is
	 *only to be used for selective rendering. It does not change the GL context
	 *to render to.
	 */
	void render(const std::string& camera_name);

	/**
	 * @brief Return the pose of the camera in the parent frame
	 * @param camera_name Camera name.
	 * @param ret_position Position of the camera.
	 * @param ret_vertical Up vector for the camera.
	 * @param ret_lookat Point the camera is looking at.
	 */
	void getCameraPoseInternal(const std::string& camera_name,
							   Eigen::Vector3d& ret_position,
							   Eigen::Vector3d& ret_vertical,
							   Eigen::Vector3d& ret_lookat);

	/**
	 * @brief Sets the pose of the camera in the parent frame
	 * @param camera_name Camera name.
	 * @param position Position of the camera.
	 * @param vertical Up vector for the camera.
	 * @param lookat Point the camera is to look at.
	 */
	void setCameraPoseInternal(const std::string& camera_name,
							   const Eigen::Vector3d& position,
							   const Eigen::Vector3d& vertical,
							   const Eigen::Vector3d& lookat);

	/* CHAI specific interface */
	/**
	 * @brief Get pointer to Chai camera object.
	 * @param camera_name Camera name.
	 */
	chai3d::cCamera* getCamera(const std::string& camera_name);

	/**
	 * @brief find the link object in the parent link recursively (called by the
	 * findLink function)
	 *
	 * @param parent parent link
	 * @param link_name name of the link to find
	 * @return chai3d::cRobotLink* pointer to the found link (or null pointer if
	 * not found)
	 */
	chai3d::cRobotLink* findLinkObjectInParentLinkRecursive(
		chai3d::cRobotLink* parent, const std::string& link_name);

	/**
	 * @brief find the link defined by the robot and link name
	 *
	 * @param robot_name the name of the robot in which to find the link
	 * @param link_name the name of the link to find
	 * @return chai3d::cRobotLink* the pointer to the link object (the function
	 * throws an error if the link is not found)
	 */
	chai3d::cRobotLink* findLink(const std::string& robot_name,
								 const std::string& link_name);

	/**
	 * @brief Enable or disable the rendering of the link frames in a given object and its children
	 * 
	 * @param parent the initial object on which to show the frames
	 * @param show_frame whether to show or hide the frames
	 * @param frame_pointer_length the length of the arrow representing the frame
	 */
	void showLinkFrameRecursive(chai3d::cRobotLink* parent, bool show_frame,
								const double frame_pointer_length);

	/**
	 * @brief finds a given force sensor display object the vector of force sensor
	 * 
	 * @param robot_or_object_name the name of the robot or object to which the sensor is attached
	 * @param link_name the name of the link to which the sensor is attached
	 * @return int the index of the force sensor display object in the vector (or -1 if not found)
	 */
	int findForceSensorDisplay(const std::string& robot_or_object_name,
							   const std::string& link_name) const;

	/// @brief pointer to the chai3d world
	chai3d::cWorld* _world;

	/// @brief pointer to the glfw window
	GLFWwindow* _window;

	/**
	 * @brief the widgets responsible for handling the computation of joint
	 * torques when right clicking and dragging the mouse on the display window
	 *
	 */
	std::vector<std::shared_ptr<UIForceWidget>> _ui_force_widgets;

	/// @brief flag to know if a right click interaction is occurring
	bool _right_click_interaction_occurring;

	/// @brief maps from robot names to filename
	std::map<std::string, std::string> _robot_filenames;
	/// @brief maps from robot names to robot models
	std::map<std::string, std::shared_ptr<SaiModel::SaiModel>> _robot_models;
	/// @brief maps from dynamic object names to pose
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>> _dyn_objects_pose;
	/// @brief maps from dynamic object names to velocity
	std::map<std::string, std::shared_ptr<Eigen::Vector6d>> _object_velocities;
	/// @brief maps from static object names to pose
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>>
		_static_objects_pose;

	/// @brief vector of force sensor displays
	std::vector<std::shared_ptr<ForceSensorDisplay>> _force_sensor_displays;

	/// @brief vector of camera names in the world
	std::vector<std::string> _camera_names;
	/// @brief index of the current camera beind rendered in the window
	int _current_camera_index;
	/// @brief maps from camera names to frame buffers for headless rendering
	std::map<std::string, chai3d::cFrameBufferPtr> _camera_frame_buffers;
	/// @brief maps from camera names to camera link attachments cameras attached to an object or robot
	std::map<std::string, std::shared_ptr<CameraLinkAttachment>>
		_camera_link_attachments;

	/// @brief last cursor x position
	double _last_cursorx;
	/// @brief last cursor y position
	double _last_cursory;
	/// @brief width of the window
	int _window_width;
	/// @brief height of the window
	int _window_height;
};

}  // namespace SaiGraphics

#endif	// CHAI_GRAPHICS_H
