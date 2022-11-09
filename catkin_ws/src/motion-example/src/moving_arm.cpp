#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "moving_arm");

  // start a background "spinner", so our node can process ROS messages
  //  - this lets us know when the move is completed
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
//  group.setRandomTarget();
  Eigen::Affine3d pose = Eigen::Translation3d(0.5, 0.0, 1.0)*Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  group.setPoseTarget(pose);
//  group.move();

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "r_wrist_roll_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "r_wrist_roll_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose obj_pose;
  obj_pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.5;
  primitive.dimensions[1] = 0.5;
  primitive.dimensions[2] = 0.5;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(obj_pose);

  attached_object.object.operation = attached_object.object.ADD;

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  kinematic_state->setToRandomPositions(joint_model_group);

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");

  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
}
