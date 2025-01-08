/**
 * \file UrdfToSaiGraphics.cpp
 *
 *  Created on: Dec 30, 2016
 *      Author: Shameek Ganguly
 */

#include "UrdfToSaiGraphics.h"

#include <urdf/urdfdom/urdf_parser/include/urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_headers/urdf_model/include/urdf_model/model.h>

#include "parser/SaiModelParserUtils.h"

typedef my_shared_ptr<SaiUrdfreader::Link> LinkPtr;
typedef const my_shared_ptr<const SaiUrdfreader::Link> ConstLinkPtr;
typedef my_shared_ptr<SaiUrdfreader::Joint> JointPtr;
typedef my_shared_ptr<SaiUrdfreader::ModelInterface> ModelPtr;
typedef my_shared_ptr<SaiUrdfreader::World> WorldPtr;

#include <Eigen/Core>
using namespace Eigen;

#include <assert.h>

#include <fstream>
#include <iostream>
#include <map>
#include <stack>
#include <vector>
using namespace std;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr> URDFLinkMap;
typedef map<string, JointPtr> URDFJointMap;

using namespace chai3d;

namespace Parser {
// internal helper function to get a child object in the tree
static cGenericObject* getGenericObjectChildRecursive(
	const std::string child_name, cGenericObject* parent) {
	if (parent->getNumChildren() == 0) {
		return NULL;
	}
	for (unsigned int i = 0; i < parent->getNumChildren(); ++i) {
		auto child = parent->getChild(i);
		if (child->m_name == child_name) {
			return child;
		}
		auto descendent = getGenericObjectChildRecursive(child_name, child);
		if (NULL != descendent) {
			return descendent;
		}
	}
	return NULL;
}

// internal helper function to load a SaiUrdfreader::Visual to a cGenericObject
// TODO: working dir default should be "", but this requires checking
// to make sure that the directory path has a trailing backslash
static void loadVisualtoGenericObject(
	cGenericObject* object,
	const my_shared_ptr<SaiUrdfreader::Visual>& visual_ptr,
	const std::string& working_dirname = "./") {
	// parse material if specified
	const auto material_ptr = visual_ptr->material;
	cColorf* color = NULL;
	if (material_ptr) {
		color = new cColorf(material_ptr->color.r, material_ptr->color.g,
							material_ptr->color.b, material_ptr->color.a);
	}
	// parse geometry if specified
	const auto geom_type = visual_ptr->geometry->type;
	auto tmp_mmesh = new cMultiMesh();
	auto tmp_mesh = new cMesh();
	if(color && color->getA() < 1.0){
		tmp_mesh->setUseTransparency(true);
	}
	if (geom_type == SaiUrdfreader::Geometry::MESH) {
		// downcast geometry ptr to mesh type
		const auto mesh_ptr = dynamic_cast<const SaiUrdfreader::Mesh*>(
			visual_ptr->geometry.get());
		assert(mesh_ptr);

		// load object
		bool file_load_success = false;

		std::string processed_filepath =
			working_dirname + "/" + mesh_ptr->filename;
		if (SaiModel::ReplaceUrdfPathPrefix(mesh_ptr->filename) !=
			mesh_ptr->filename) {
			processed_filepath =
				SaiModel::ReplaceUrdfPathPrefix(mesh_ptr->filename);
		}

		if (processed_filepath.length() < 5) {
			cerr << "Couldn't load obj/3ds/STL robot link file, extension not "
					"supported: "
				 << processed_filepath << endl;
			abort();
		}

		std::string extension =
			processed_filepath.substr(processed_filepath.length() - 4);
		std::transform(extension.begin(), extension.end(), extension.begin(),
					   [](unsigned char c) { return std::tolower(c); });

		if (extension == ".stl") {
			file_load_success = cLoadFileSTL(tmp_mmesh, processed_filepath);
		} else if (extension == ".obj") {
			file_load_success = cLoadFileOBJ(tmp_mmesh, processed_filepath);
		} else if (extension == ".3ds") {
			file_load_success = cLoadFile3DS(tmp_mmesh, processed_filepath);
		}
		if (!file_load_success) {
			cerr << "Couldn't load obj/3ds/STL robot link file: "
				 << processed_filepath << endl;
			abort();
		}

		// apply scale
		tmp_mmesh->scaleXYZ(mesh_ptr->scale.x, mesh_ptr->scale.y,
							mesh_ptr->scale.z);
		if (color) {
			tmp_mmesh->m_material->setColor(*color);
		}
	} else if (geom_type == SaiUrdfreader::Geometry::BOX) {
		// downcast geometry ptr to box type
		const auto box_ptr = dynamic_cast<const SaiUrdfreader::Box*>(
			visual_ptr->geometry.get());
		assert(box_ptr);
		// create chai box mesh
		cCreateBox(tmp_mesh, box_ptr->dim.x, box_ptr->dim.y, box_ptr->dim.z);
		if (color) {
			tmp_mesh->m_material->setColor(*color);
		}
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == SaiUrdfreader::Geometry::SPHERE) {
		// downcast geometry ptr to sphere type
		const auto sphere_ptr = dynamic_cast<const SaiUrdfreader::Sphere*>(
			visual_ptr->geometry.get());
		assert(sphere_ptr);
		// create chai sphere mesh
		cCreateSphere(tmp_mesh, sphere_ptr->radius);
		if (color) {
			tmp_mesh->m_material->setColor(*color);
		}
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == SaiUrdfreader::Geometry::CYLINDER) {
		// downcast geometry ptr to cylinder type
		const auto cylinder_ptr = dynamic_cast<const SaiUrdfreader::Cylinder*>(
			visual_ptr->geometry.get());
		assert(cylinder_ptr);
		// create chai cylinder mesh
		chai3d::cCreateCylinder(tmp_mesh, cylinder_ptr->length,
								cylinder_ptr->radius, 32, 1, 1, true, true,
								cVector3d(0, 0, -cylinder_ptr->length / 2));
		if (color) {
			tmp_mesh->m_material->setColor(*color);
		}
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == SaiUrdfreader::Geometry::CAPSULE) {
		// downcast geometry ptr to cylinder type
		const auto capsule_ptr = dynamic_cast<const SaiUrdfreader::Capsule*>(
			visual_ptr->geometry.get());
		assert(capsule_ptr);
		if (color) {
			if (material_ptr && material_ptr->has_color2) {
				auto color2 =
					new cColorf(material_ptr->color2.r, material_ptr->color2.g,
								material_ptr->color2.b, material_ptr->color2.a);
				chai3d::cCreateCapsule(tmp_mesh, capsule_ptr->radius,
									   capsule_ptr->length,
									   20, /* a_num_longitudinal_slices */
									   16, /* a_num_circumferential_slices */
									   *color, *color2);
				// if dual color information is present, we use the bicolor
				// gradient rendering of the capsule which requires vertex
				// coloring
				tmp_mesh->setUseVertexColors(true, true);
			} else {
				chai3d::cCreateCapsule(tmp_mesh, capsule_ptr->radius,
									   capsule_ptr->length);
				// vertex coloring not needed, we set the material property
				tmp_mesh->m_material->setColor(*color);
			}
		} else {
			chai3d::cCreateCapsule(tmp_mesh, capsule_ptr->radius,
								   capsule_ptr->length);
			// if no color information is present, we use the default rendering
			// of the capsule which has a vertex coloring based bicolor gradient
			tmp_mesh->setUseVertexColors(true, true);
		}
		tmp_mmesh->addMesh(tmp_mesh);
	} else if (geom_type == SaiUrdfreader::Geometry::PYRAMID) {
		// downcast geometry ptr to pyramid type
		const auto pyramid_ptr = dynamic_cast<const SaiUrdfreader::Pyramid*>(
			visual_ptr->geometry.get());
		assert(pyramid_ptr);
		chai3d::cCreatePyramid(tmp_mesh, pyramid_ptr->num_sides,
							   pyramid_ptr->base_size, pyramid_ptr->height,
							   pyramid_ptr->use_center_vertex);
		if (color) {
			tmp_mesh->m_material->setColor(*color);
		}
		tmp_mmesh->addMesh(tmp_mesh);
	}
	if (color) {
		delete color;
	}
	// set position and orientation for mesh
	tmp_mmesh->setLocalPos(cVector3d(visual_ptr->origin.position.x,
									 visual_ptr->origin.position.y,
									 visual_ptr->origin.position.z));
	{  // brace temp variables to separate scope
		auto urdf_q = visual_ptr->origin.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3;
		tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());
		tmp_mmesh->setLocalRot(tmp_cmat3);
	}
	// create brute force collision detector
	tmp_mmesh->createBruteForceCollisionDetector();
	tmp_mmesh->m_name = object->m_name;
	// add as child to object model
	object->addChild(tmp_mmesh);
}

void UrdfToSaiGraphicsWorld(
	const std::string& filename, chai3d::cWorld* world,
	std::map<std::string, std::string>& robot_filenames,
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>>& dyn_object_poses,
	std::map<std::string, std::shared_ptr<Eigen::Affine3d>>&
		static_object_poses,
	std::map<std::string, cFrameBufferPtr>& camera_frame_buffers,
	bool verbose) {
	// load world urdf file
	std::string resolved_filename = SaiModel::ReplaceUrdfPathPrefix(filename);
	ifstream model_file(resolved_filename);
	if (!model_file) {
		cerr << "Error opening file '" << resolved_filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
							std::istreambuf_iterator<char>());

	model_file.close();

	// parse xml to URDF world model
	assert(world);
	WorldPtr urdf_world = SaiUrdfreader::parseURDFWorld(model_xml_string);
	world->m_name = urdf_world->name_;
	if (verbose) {
		cout << "UrdfToSaiGraphicsWorld: Starting model conversion to chai "
				"graphics world."
			 << endl;
		cout << "+ add world: " << world->m_name << endl;
	}

	// parse robots
	for (const auto robot_spec_pair : urdf_world->models_) {
		const auto robot_spec = robot_spec_pair.second;

		// get translation
		auto tmp_cvec3 = cVector3d(robot_spec->origin.position.x,
								   robot_spec->origin.position.y,
								   robot_spec->origin.position.z);
		// get rotation
		auto urdf_q = robot_spec->origin.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3;
		tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

		// create new robot base object represented by a cRobotBase
		cRobotBase* robot = new cRobotBase();
		robot->setLocalPos(tmp_cvec3);
		robot->setLocalRot(tmp_cmat3);
		world->addChild(robot);

		// load robot from file
		UrdfToSaiGraphicsRobot(
			robot_spec->model_filename, robot, verbose,
			SaiModel::ReplaceUrdfPathPrefix(robot_spec->model_working_dir));
		assert(robot->m_name == robot_spec->model_name);

		// overwrite robot name with custom name for this instance
		robot->m_name = robot_spec->name;

		// fill robot filenames
		auto it = robot_filenames.find(robot->m_name);
		if (it != robot_filenames.end()) {
			throw std::runtime_error(
				"Different robots cannot have the same name in the world");
		}
		robot_filenames[robot->m_name] =
			SaiModel::ReplaceUrdfPathPrefix(robot_spec->model_working_dir) +
			"/" + robot_spec->model_filename;
	}

	// parse cameras
	for (const auto camera_pair : urdf_world->graphics_.cameras) {
		const auto camera_ptr = camera_pair.second;
		auto it = camera_frame_buffers.find(camera_ptr->name);
		if (it != camera_frame_buffers.end()) {
			throw std::runtime_error(
				"Different cameras cannot have the same name in the world");
		}

		// initialize a chai camera
		cCamera* camera = new cCamera(world);
		cFrameBufferPtr fb = cFrameBuffer::create();
		fb->setup(camera);
		camera_frame_buffers[camera_ptr->name] = fb;
		// TODO: support link mounted camera
		// name camera
		camera->m_name = camera_ptr->name;
		// add camera properties
		world->addChild(camera);

		// position and orient the camera
		cVector3d camera_position(camera_ptr->position.x,
								  camera_ptr->position.y,
								  camera_ptr->position.z);
		cVector3d camera_lookat(camera_ptr->lookat.x, camera_ptr->lookat.y,
								camera_ptr->lookat.z);
		cVector3d camera_up(camera_ptr->vertical.x, camera_ptr->vertical.y,
							camera_ptr->vertical.z);
		camera->set(camera_position, camera_lookat, camera_up);

		// TODO: parse from urdf
		// set the near and far clipping planes of the camera
		camera->setClippingPlanes(0.01, 10.0);

		// TODO: parse from urdf
		// // set vertical mirrored display mode
		// camera->setMirrorVertical(false);
	}

	// parse lights
	for (const auto light_pair : urdf_world->graphics_.lights) {
		const auto light_ptr = light_pair.second;
		// initialize a chai light
		if (light_ptr->type == "directional" || light_ptr->type == "spot") {
			cDirectionalLight* light;
			if (light_ptr->type == "directional") {
				// create a directional light source
				light = new cDirectionalLight(world);
				// TODO: support link mounted light
			} else if (light_ptr->type == "spot") {
				cSpotLight* spot_light = new cSpotLight(world);
				;
				// enable shadow casting
				spot_light->setShadowMapEnabled(true);
				// up cast to cDirectionalLight
				light = dynamic_cast<cDirectionalLight*>(spot_light);
			}

			light->setLocalPos(cVector3d(light_ptr->position.x,
										 light_ptr->position.y,
										 light_ptr->position.z));

			// insert light source inside world
			world->addChild(light);
			light->m_name = light_ptr->name;

			// enable light source
			light->setEnabled(true);

			// define direction of light beam
			light->setDir(light_ptr->lookat.x - light_ptr->position.x,
						  light_ptr->lookat.y - light_ptr->position.y,
						  light_ptr->lookat.z - light_ptr->position.z);
			assert(light->getDir().length() != 0.0);
		}
		// TODO: support other chai light types
	}

	// parse static meshes
	for (const auto object_pair : urdf_world->graphics_.static_objects) {
		const auto object_ptr = object_pair.second;
		// initialize a cGenericObject to represent this object in the world
		cGenericObject* object = new cGenericObject();
		object->m_name = object_ptr->name;

		// get translation
		auto tmp_cvec3 = cVector3d(object_ptr->origin.position.x,
								   object_ptr->origin.position.y,
								   object_ptr->origin.position.z);
		// get rotation
		auto urdf_q = object_ptr->origin.rotation;
		Eigen::Quaterniond tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3;
		tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

		// object pose
		static_object_poses[object->m_name] =
			std::make_shared<Eigen::Affine3d>(tmp_q);
		static_object_poses.at(object->m_name)->translation() =
			tmp_cvec3.eigen();

		// set object position and rotation
		object->setLocalPos(tmp_cvec3);
		object->setLocalRot(tmp_cmat3);

		// add to world
		world->addChild(object);

		// load object graphics
		if (!object_ptr->visual && verbose) {
			cout << "Warning: object " << object_ptr->name
				 << " has no visual element." << endl;
		}
		for (const auto visual_ptr : object_ptr->visual_array) {
			loadVisualtoGenericObject(object, visual_ptr);
		}
	}

	// parse dynamic objects
	for (const auto object_pair : urdf_world->graphics_.dynamic_objects) {
		const auto object_ptr = object_pair.second;
		// initialize a cGenericObject to represent this object in the world
		cGenericObject* object = new cGenericObject();
		object->m_name = object_ptr->name;

		// get translation
		auto tmp_cvec3 = cVector3d(object_ptr->origin.position.x,
								   object_ptr->origin.position.y,
								   object_ptr->origin.position.z);
		// get rotation
		auto urdf_q = object_ptr->origin.rotation;
		Eigen::Quaterniond tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d tmp_cmat3;
		tmp_cmat3.copyfrom(tmp_q.toRotationMatrix());

		// initial object pose
		dyn_object_poses[object->m_name] =
			std::make_shared<Eigen::Affine3d>(tmp_q);
		dyn_object_poses.at(object->m_name)->translation() = tmp_cvec3.eigen();

		// set object position and rotation
		object->setLocalPos(tmp_cvec3);
		object->setLocalRot(tmp_cmat3);

		// add to world
		world->addChild(object);

		// load object graphics
		if (!object_ptr->visual) {
			throw std::runtime_error(
				"Dynamic object " + object_ptr->name +
				" has no visual element. Dynamic objects must have a visual "
				"element.");
		}
		for (const auto visual_ptr : object_ptr->visual_array) {
			loadVisualtoGenericObject(object, visual_ptr);
		}
	}
}

void UrdfToSaiGraphicsRobot(const std::string& filename,
							 chai3d::cRobotBase* base, bool verbose,
							 const std::string& working_dirname) {
	// load and parse model file
	string filepath = working_dirname + "/" + filename;
	ifstream model_file(filepath);
	if (!model_file) {
		cerr << "Error opening file '" << filepath << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
							std::istreambuf_iterator<char>());

	model_file.close();

	// read and parse xml string to urdf model
	assert(base);
	ModelPtr urdf_model = SaiUrdfreader::parseURDF(model_xml_string);
	base->m_name = urdf_model->getName();
	if (verbose) {
		cout << "UrdfToSaiGraphicsRobot: Starting model conversion to chai."
			 << endl;
		cout << "+ add robot: " << base->m_name << endl;
	}

	// load urdf model to dynamics3D link tree
	LinkPtr urdf_root_link;

	URDFLinkMap link_map;  // map<string, LinkPtr >
	link_map = urdf_model->links_;

	URDFJointMap joint_map;	 // map<string, JointPtr >
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr> link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	// push the root LinkPtr to link stack. link stack height = 1
	// NOTE: depth first search happens due to use of stack
	link_stack.push(link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot();

	// TODO: Not sure if this is ever expected to be true or not
	// 	Mikael's example URDF has it set to false
	if (root->visual) {
		// initialize a cRobotLink for the root link
		cRobotLink* root_object = new cRobotLink();
		root_object->m_name = root->name;

		// add to base
		base->addChild(root_object);

		// parse visual meshes
		for (const auto visual_ptr : root->visual_array) {
			loadVisualtoGenericObject(root_object, visual_ptr, working_dirname);
		}

		if (verbose) {
			cout << "+ Adding Root Body to chai render" << endl;
			cout << "  body name   : " << root_object->m_name << endl;
		}
	}  // endif (root->visual)

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0);	// SG: what does this do??
	} else {
		cerr << "Base link has no associated joints!" << endl;
		abort();
	}

	// this while loop is to enumerate all joints in the tree structure by name
	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		// if there are unvisited child joints on current link:
		// 	then add link to stack
		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push(joint_idx + 1);

			// SG: the URDF model structure is:
			//	every non-terminal link has child joint(s)
			// 	every joint has child link (else, we would get an exception
			// right below)
			link_stack.push(link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			if (verbose) {
				for (unsigned int i = 1; i < joint_index_stack.size() - 1;
					 i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '"
					 << link_stack.top()->name << "' type = " << cur_joint->type
					 << endl;
			}

			joint_names.push_back(cur_joint->name);
			// SG: this is the only data structure of interest it seems
			// all joints are processed in the for loop below
		} else {  // else this link has been processed, so pop link
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	// iterate over all joints
	for (unsigned int j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
		LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

		// determine where to add the current joint and child body
		cRobotLink* parent_link = NULL;
		parent_link = dynamic_cast<cRobotLink*>(getGenericObjectChildRecursive(
			urdf_parent->name, base));	// returns NULL if link does not exist

		// cout << "joint: " << urdf_joint->name << "\tparent = " <<
		// urdf_parent->name << " child = " << urdf_child->name << " parent_id =
		// " << rbdl_parent_id << endl;

		// create a new link
		cRobotLink* link = new cRobotLink();
		link->m_name = urdf_child->name;

		// load visuals
		for (const auto visual_ptr : urdf_child->visual_array) {
			loadVisualtoGenericObject(link, visual_ptr, working_dirname);
		}

		// compute the joint transformation which acts as the child link
		// transform with respect to the parent
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY(
			joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation
			<< urdf_joint->parent_to_joint_origin_transform.position.x,
			urdf_joint->parent_to_joint_origin_transform.position.y,
			urdf_joint->parent_to_joint_origin_transform.position.z;
		auto urdf_q = urdf_joint->parent_to_joint_origin_transform.rotation;
		Quaternion<double> tmp_q(urdf_q.w, urdf_q.x, urdf_q.y, urdf_q.z);
		cMatrix3d rot_in_parent;
		rot_in_parent.copyfrom(tmp_q.toRotationMatrix());
		link->setLocalPos(joint_translation);
		link->setLocalRot(rot_in_parent);

		// add to parent link
		if (NULL == parent_link) {
			// link is located at root
			base->addChild(link);
		} else {
			// link to parent
			parent_link->addChild(link);
		}

		if (verbose) {
			cout << "+ Adding Body " << endl;
			if (NULL == parent_link) {
				cout << "  parent_link_name  : NULL" << endl;
			} else {
				cout << "  parent_link_name  : " << parent_link->m_name << endl;
			}
			cout << "  position in parent: " << joint_translation.transpose()
				 << endl;
			cout << "  orientation in parent: " << tmp_q.coeffs().transpose()
				 << endl;
			cout << "  body name   : " << link->m_name << endl;
		}
	}

	if (verbose) {
		cout << "UrdfToSaiGraphicsRobot: Finished model conversion to chai."
			 << endl;
	}
}

}  // namespace Parser
