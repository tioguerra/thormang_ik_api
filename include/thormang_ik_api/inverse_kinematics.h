#ifndef THORMANG_IK_API_H
#define THROMANG_IK_API_H

#include <vector>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

void parseWeightData(const std::string &path);

bool calc_ik(thormang_ik_api::calc_ik::Request &req,
             thormang_ik_api::calc_ik::Response &res);

thormang3::KinematicsDynamics *robotis_;

/* inverse kinematics */
Eigen::MatrixXd ik_weight_;

#endif

