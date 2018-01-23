#include "ros/ros.h"
#include "thormang_ik_api/calc_ik.h"
#include "thormang_ik_api/inverse_kinematics.h"

void parseWeightData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    ROS_INFO("Trying to load YAML file");
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node ik_weight_node = doc["weight_value"];
  for (YAML::iterator it = ik_weight_node.begin(); it != ik_weight_node.end(); ++it)
  {
    int     id    = it->first.as<int>();
    double  value = it->second.as<double>();

    ik_weight_.coeffRef(id, 0) = value;
    ROS_INFO("ik_weight_.coeffRef(%d, 0) = %f", id, value);
  }
}

bool calc_ik(thormang_ik_api::calc_ik::Request &req,
             thormang_ik_api::calc_ik::Response &res)
{
  ROS_INFO("Calculating inverse kinematics now.");

  // Initialize the constants just like Robotis does in manipulation_module.cpp
  int max_iter = 30;
  double ik_tol = 1e-2;
  int ik_id_start_ = 0;
  int ik_id_end_ = 0;

  // This string in the request should specify which arm to use
  if (req.whichArm.data == "left")
  {
    ROS_INFO("Calculating Inverse Kinematics for left arm.");
    ik_id_start_  = ID_L_ARM_START;
    ik_id_end_    = ID_L_ARM_END;
  } else {
    ROS_INFO("Calculating Inverse Kinematics for right arm.");
    ik_id_start_  = ID_R_ARM_START;
    ik_id_end_    = ID_R_ARM_END;
  }

  // Copy the current robot pose from the request. Here we expect the name and value pairs
  // to correspond to joint names and angle values for the left or right arm, to update
  // the current robot pose for the Jacobian based inverse kinematics
  ROS_INFO("Copying current robot pose.");
  for (auto jointPose = req.currJointPose.begin(); jointPose != req.currJointPose.end(); jointPose++)
  {
    for (int id = 1 ; id <= MAX_JOINT_ID ; id++)
    {
      if (robotis_->thormang3_link_data_[id]->name_ == jointPose->name)
      {
        ROS_INFO("Found joint %s", jointPose->name.c_str());
        robotis_->thormang3_link_data_[id]->joint_angle_ = jointPose->value;
      }
    }
  }
  
  ROS_INFO("Creating desired position.");
  Eigen::MatrixXd ik_target_position_  = Eigen::MatrixXd::Zero(3,1);
  ik_target_position_.coeffRef(0, 0) = req.desiredPose.position.x;
  ik_target_position_.coeffRef(1, 0) = req.desiredPose.position.y;
  ik_target_position_.coeffRef(2, 0) = req.desiredPose.position.z;

  ROS_INFO("Creating desired orientation.");
  Eigen::MatrixXd ik_target_rotation_  = Eigen::MatrixXd::Zero(4,1);
  ik_target_rotation_.coeffRef(0, 0) = req.desiredPose.orientation.x;
  ik_target_rotation_.coeffRef(1, 0) = req.desiredPose.orientation.y;
  ik_target_rotation_.coeffRef(2, 0) = req.desiredPose.orientation.z;
  ik_target_rotation_.coeffRef(3, 0) = req.desiredPose.orientation.w;

  ROS_INFO("Calling the ROBOTIS calcInverseKinematics() function.");
  // Here we calculate the inverse kinematics
  bool ik_success = robotis_->calcInverseKinematics(ik_id_start_, ik_id_end_,
          ik_target_position_, ik_target_rotation_,
          max_iter, ik_tol, ik_weight_);

  // Here we handle with the results
  if (ik_success == true)
  {
    ROS_INFO("Success! Finding route.");
    std::vector<int> idx = robotis_->findRoute(ik_id_start_, ik_id_end_);
    ROS_INFO("Copying the resulting target angles to the joints in the route.");
    for (auto i = idx.begin() ; i != idx.end() ; i++)
    {
      thormang3_manipulation_module_msgs::JointPose jointPose;
      jointPose.name = robotis_->thormang3_link_data_[*i]->name_;
      jointPose.value = robotis_->thormang3_link_data_[*i]->joint_angle_;
      res.targJointPose.push_back(jointPose);
    }
  } else {
    ROS_ERROR("Inverse Kinematics Failed!!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematics_server");
  ros::NodeHandle n;

  // Setup
  ROS_INFO("Loading link weight info");
  ik_weight_            = Eigen::MatrixXd::Zero(MAX_JOINT_ID+1, 1);
  ik_weight_.fill(1.0);
  std::string _path = ros::package::getPath("thormang_ik_api") + "/config/ik_weight.yaml";
  parseWeightData(_path);
  
  ROS_INFO("Creating the KinematicsDynamics object");
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
  // These are declared in thormang3_kinematics_dynamics/kinematics_dynamics.h

  ros::ServiceServer service = n.advertiseService("thormang_ik_api/calc_ik", calc_ik);
  ROS_INFO("The inverse_kinematics service is ready!");

  ros::spin();

  return 0;
}

