#ifndef SaiGraphics_UIFORCE_WIDGET_H
#define SaiGraphics_UIFORCE_WIDGET_H

#include <SaiModel.h>
#include <chai3d.h>

#include <Eigen/Core>
#include <string>

#include "chai_extension/CRobotLink.h"

namespace SaiGraphics {

/**
 * @brief A class to enable the application a force or moment to a robot or
 * object object in the world by detecting which point on the robot/object is
 * under the cursor when turned on (for example when clicking) and computing a
 * force or moment proportional to the drag distance, with velocity based
 * damping. The force/moment is rendered as a green/red line in the world.
 */
class UIForceWidget {
public:
	/**
	 * @brief State of the UIForceWidget
	 *
	 */
	enum UIForceWidgetState { Disabled = 0, Inactive, Active };

public:
	/**
	 * @brief Construct a new UIForceWidget object for a robot
	 *
	 * @param robot_name name of the robot to which the widget is attached
	 * @param interact_at_object_center whether to apply the force at the
	 * clicked link center, or on the clicked point
	 * @param robot robot model of the robot to which the widget is attached
	 * (used to access robot pose and velocity)
	 * @param display_line a line to be displayed when an interaction force is
	 * applied
	 */
	UIForceWidget(const std::string &robot_name,
				  const bool interact_at_object_center,
				  std::shared_ptr<SaiModel::SaiModel> robot,
				  chai3d::cShapeLine *display_line);

	/**
	 * @brief Construct a new UIForceWidget object for an object
	 *
	 * @param object_name name of the object to which the widget is attached
	 * @param interact_at_object_center whether to apply the force at the object
	 * center, or on the clicked point
	 * @param object_pose pose of the object to which the widget is attached
	 * (used for reading purposes only)
	 * @param object_velocity velocity of the object to which the widget is
	 * attached (used for reading purposes only)
	 * @param display_line a line to be displayed when an interaction force is
	 * applied
	 */
	UIForceWidget(const std::string &object_name,
				  const bool interact_at_object_center,
				  std::shared_ptr<Eigen::Affine3d> object_pose,
				  std::shared_ptr<Eigen::Vector6d> object_velocity,
				  chai3d::cShapeLine *display_line);

	/**
	 * @brief Setter to enable or disable the widget
	 *
	 * @param enable
	 */
	void setEnable(bool enable);

	/**
	 * @brief Setter for the virtual spring damper parameters to generate the
	 * force
	 *
	 * @param linear_stiffness linear stiffness
	 * @param rotational_stiffness rotational stiffness
	 * @param linear_damping linear damping
	 * @param rotational_damping rotational damping
	 */
	void setNominalSpringParameters(const double linear_stiffness,
									const double rotational_stiffness,
									const double linear_damping,
									const double rotational_damping) {
		_linear_stiffness = linear_stiffness;
		_rotational_stiffness = rotational_stiffness;
		_linear_damping = linear_damping;
		_rotational_damping = rotational_damping;
	}

	/**
	 * @brief Getter for the state of the widget
	 *
	 * @return UIForceWidgetState the state of the widget
	 */
	UIForceWidgetState getState() const { return _state; }

	/**
	 * @brief This function should be called when the click is pressed. It will
	 * detect if the robot or object is under the cursor, is fo, enble the force
	 * widget and record the initial click position, and start computing the
	 * force/moment to apply. It should be continuously called while the click is held.
	 *
	 * @param camera the camera object in the chai world
	 * @param viewx x-position of cursor in viewport (OpenGL style screen co-ordinates)
	 * @param viewy y-position of cursor in viewport (OpenGL style screen co-ordinates)
	 * @param window_width width of viewport in screen co-ordinates
	 * @param window_height height of viewport in screen co-ordinates
	 * @param depth_change change in depth of the click point
	 * @return true if the object is under the cursor when initially clicked, false otherwise
	 */
	bool setInteractionParams(chai3d::cCamera *camera, int viewx, int viewy,
							  int window_width, int window_height,
							  double depth_change);

	/**
	 * @brief Set the widget to apply a force
	 * 
	 */
	void setForceMode();

	/**
	 * @brief Set the widget to apply a moment
	 * 
	 */
	void setMomentMode();

	/**
	 * @brief Getter for the force or moment mode
	 * 
	 * @return true if the widget is in force mode, false if in moment mode
	 */
	bool isForceMode() const { return _force_mode; }

	/**
	 * @brief Get the Applied Force Moment object
	 * 
	 * @return Eigen::Vector6d the force/moment applied by the widget
	 */
	Eigen::Vector6d getAppliedForceMoment() const;

	/**
	 * @brief Get the UI Joint Torques corresponding to the applied force/moment
	 * 
	 * @return Eigen::VectorXd the joint torques due to the applied force/moment
	 */
	Eigen::VectorXd getUIJointTorques() const;

	/**
	 * @brief Get the Robot Or Object Name
	 * 
	 * @return const std::string the name of the robot or object to which the widget is attached
	 */
	const std::string getRobotOrObjectName() { return _robot_or_object_name; }

private:
	/**
	 * @brief Get info about link of the robot at the given cursor position.
	 * @return True if a link is present, False if not
	 * @param camera_name Camera name.
	 * @param robot_name Name of robot to look for.
	 * @param view_x x-position of cursor in viewport (OpenGL style screen
	 * co-ordinates).
	 * @param view_y y-position of cursor in viewport (OpenGL style screen
	 * co-ordinates).
	 * @param window_width Width of viewport in screen co-ordinates.
	 * @param window_height Height of viewport in screen co-ordinates.
	 * @param ret_link_name Name of the link. Garbage if no link present at
	 * cursor location.
	 * @param ret_pos Position of cursor in link frame. Garbage if no link
	 * present at cursor location.
	 */
	bool getRobotLinkInCamera(chai3d::cCamera *camera, int view_x, int view_y,
							  int window_width, int window_height,
							  std::string &ret_link_name,
							  Eigen::Vector3d &ret_pos);

	/**
	 * @brief Initialize all the internal parameters of the widget
	 * 
	 */
	void internalInit();

	/// @brief a line to be displayed when an interaction force is applied
	chai3d::cShapeLine *_display_line;

	/// @brief name of the robot or object to which the widget is attached
	std::string _robot_or_object_name;

	/// @brief pointer to the robot model (if the sensor is attached to a robot)
	std::shared_ptr<SaiModel::SaiModel> _robot;

	/// @brief pose of the object to which the widget is attached (if the sensor is attached to an object)
	std::shared_ptr<Eigen::Affine3d> _object_pose;

	/// @brief velocity of the object to which the widget is attached (if the sensor is attached to an object)
	std::shared_ptr<Eigen::Vector6d> _object_velocity;

	/// @brief flag to know if the widget is attached to a robot or object
	bool _is_robot;

	/// @brief State of the UIForceWidget
	UIForceWidgetState _state;

	/// @brief linear stiffness to compute the force
	double _linear_stiffness;
	/// @brief rotational stiffness to compute the moment
	double _rotational_stiffness;
	/// @brief linear damping to compute the force
	double _linear_damping;
	/// @brief rotational damping to compute the moment
	double _rotational_damping;

	/// @brief maximum allowable force
	double _max_force;
	/// @brief maximum allowable moment
	double _max_moment;

	/// @brief flag to know if we want to apply a force or moment
	bool _force_mode;

	/// @brief name of the link to which the force is being applied
	std::string _link_name;

	/// @brief local position at which force is being applied
	Eigen::Vector3d _link_local_pos;

	/// @brief initial position of the point that was clicked
	Eigen::Vector3d _initial_click_point;

	/// @brief flag to know if the force is being applied at the object center or at the clicked point
	bool _interact_at_object_center;

	/// @brief depth of the click point
	double _click_depth;
};

}  // namespace SaiGraphics

#endif	// SaiGraphics_UIFORCE_WIDGET_H