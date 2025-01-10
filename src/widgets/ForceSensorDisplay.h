#ifndef SaiGraphics_FORCE_SENSOR_DISPLAY_H
#define SaiGraphics_FORCE_SENSOR_DISPLAY_H

#include <SaiModel.h>
#include <chai3d.h>

namespace SaiGraphics {

/**
 * @brief Class to display force and moments sensed by a force/moment sensor.
 * This will display the reaction forces/moments on the sensor as a green line
 * for the force and a red/brown line for the moment. The lines start at the
 * sensor location in the rendered world. For example, if the sensor is pushing
 * down on the floor, the displayed line for the force will go up.
 */
class ForceSensorDisplay {
public:
	/**
	 * @brief Construct a new Force Sensor Display object for a simulated robot
	 *
	 * @param robot_name the name of the robot on which the sensor is attached
	 * @param link_name the name of the link on which the sensor is attached
	 * @param T_link_sensor the transform from the link frame to the sensor
	 * frame
	 * @param robot pointer to the robot model (the robot model is not modified
	 * internally, this pointer is for reading robot state only)
	 * @param chai_world pointer to the chai3d world
	 */
	ForceSensorDisplay(const std::string& robot_name,
					   const std::string& link_name,
					   const Eigen::Affine3d T_link_sensor,
					   std::shared_ptr<SaiModel::SaiModel> robot,
					   chai3d::cWorld* chai_world);

	/**
	 * @brief Construct a new Force Sensor Display object for a simulated object
	 *
	 * @param object_name the name of the object on which the sensor is
	 * attached
	 * @param link_name the name of the link on which the sensor is attached
	 * @param T_link_sensor the transform from the object frame to the sensor
	 * frame
	 * @param object_pose pointer to the object pose (the object pose is not
	 * modified internally, this pointer is for reading object state only)
	 * @param chai_world pointer to the chai3d world
	 */
	ForceSensorDisplay(const std::string& object_name,
					   const std::string& link_name,
					   const Eigen::Affine3d T_link_sensor,
					   std::shared_ptr<Affine3d> object_pose,
					   chai3d::cWorld* chai_world);

	/**
	 * @brief Updates the force and moment lines displayed in the world
	 * according to the latest values of the force and moment.
	 *
	 * @param force_global_frame The force to display in the global frame
	 * @param moment_global_frame The moment to display in the global frame
	 */
	void update(const Eigen::Vector3d& force_global_frame,
				const Eigen::Vector3d& moment_global_frame);

	/**
	 * @brief Getter for the name of robot or object to which the sensor is
	 * attached
	 *
	 * @return std::string the name of the robot or object
	 */
	std::string robot_or_object_name() const { return _robot_or_object_name; }

	/**
	 * @brief Getter for the name of the link to which the sensor is attached
	 * (it will be the default object link name if the sensor is on an object)
	 *
	 * @return std::string the name of the link
	 */
	std::string link_name() const { return _link_name; }

	/**
	 * @brief Getter for the transform from the link frame to the sensor frame
	 *
	 * @return Eigen::Affine3d the transform from the link frame to the sensor
	 */
	Eigen::Affine3d T_link_sensor() const { return _T_link_sensor; }

private:
	/**
	 * @brief Initialize the display lines for the force and moment, and adds
	 * them to the chai3d world
	 *
	 * @param chai_world pointer to the chai3d world
	 */
	void initializeLines(chai3d::cWorld* chai_world);

	/// @brief a line to be displayed when a contact force is active
	chai3d::cShapeLine* _display_line_force;

	/// @brief a line to be displayed when a contact moment is active
	chai3d::cShapeLine* _display_line_moment;

	/// @brief pointer to the robot model (if the sensor is attached to a robot)
	std::shared_ptr<SaiModel::SaiModel> _robot;

	/// @brief name of the robot or object to which the sensor is attached
	const std::string _robot_or_object_name;

	/// @brief name of the link to which the sensor is attached
	const std::string _link_name;

	/// @brief transform from link to sensor frame
	const Eigen::Affine3d _T_link_sensor;

	/// @brief pointer to the object pose (if the sensor is attached to an object)
	std::shared_ptr<Eigen::Affine3d> _object_pose;

	/// @brief scale of the force line displayed from 0 to 1
	double _force_line_scale;

	/// @brief scale of the moment line displayed from 0 to 1
	double _moment_line_scale;
};

}  // namespace SaiGraphics

#endif	// SaiGraphics_FORCE_SENSOR_DISPLAY_H
