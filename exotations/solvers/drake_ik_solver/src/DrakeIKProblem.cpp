/*
 * DrakeIKProblem.cpp
 *
 *  Created on: 7 Oct 2015
 *      Author: yiming
 */

#include "drake_ik_solver/DrakeIKProblem.h"

REGISTER_PROBLEM_TYPE("DrakeIKProblem", exotica::DrakeIKProblem);

namespace exotica
{
  DrakeIKProblem::DrakeIKProblem()
      : model_(NULL)
  {

  }

  DrakeIKProblem::~DrakeIKProblem()
  {
    if (model_) delete model_;
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
  }

  EReturn DrakeIKProblem::reinitialise(rapidjson::Document& document,
      boost::shared_ptr<PlanningProblem> problem)
  {
    for (int i = 0; i < constraints_.size(); i++)
      if (constraints_[i]) delete constraints_[i];
    constraints_.clear();
    Eigen::Vector2d tspan01;
    tspan01 << 0, 1;
    if (document.IsArray())
    {
      for (rapidjson::SizeType i = 0; i < document.Size(); i++)
      {
        rapidjson::Value& obj = document[i];
        if (obj.IsObject())
        {
          std::string constraintClass;
          if (ok(getJSON(obj["class"], constraintClass)))
          {
            if (constraintClass.compare("QuasiStaticConstraint") == 0)
            {
              QuasiStaticConstraint* kc_quasi = NULL;
              if (ok(buildQuasiStaticConstraint(obj, kc_quasi)))
                constraints_.push_back(kc_quasi);
              else
              {
                if (kc_quasi) delete kc_quasi;
                INDICATE_FAILURE
                return FAILURE;
              }
            }
            else if (constraintClass.compare("PostureConstraint") == 0)
            {
              PostureConstraint* kc_pos = NULL;
              if (ok(buildPostureConstraint(obj, kc_pos)))
                constraints_.push_back(kc_pos);
              else
              {
                if (kc_pos) delete kc_pos;
                INDICATE_FAILURE
                return FAILURE;
              }
            }
            else if (constraintClass.compare("PositionConstraint") == 0)
            {
              WorldPositionConstraint* kc_position = NULL;
              if (ok(buildWorldPositionConstraint(obj, kc_position)))
                constraints_.push_back(kc_position);
              else
              {
                if (kc_position) delete kc_position;
                INDICATE_FAILURE
                return FAILURE;
              }
            }
            else if (constraintClass.compare("QuatConstraint") == 0)
            {
              WorldQuatConstraint* kc_quat = NULL;
              if (ok(buildWorldQuatConstraint(obj, kc_quat)))
                constraints_.push_back(kc_quat);
              else
              {
                if (kc_quat) delete kc_quat;
                INDICATE_FAILURE
                return FAILURE;
              }
            }
            else if (constraintClass.compare("FixedLinkFromRobotPoseConstraint")
                == 0)
            {
              WorldPositionConstraint* kc_fixed_pos = NULL;
              WorldQuatConstraint* kc_fixed_quat = NULL;
              if (ok(
                  buildFixedLinkFromRobotPoseConstraint(obj, kc_fixed_pos,
                      kc_fixed_quat)))
              {
                constraints_.push_back(kc_fixed_pos);
                constraints_.push_back(kc_fixed_quat);
              }
              else
              {
                if (kc_fixed_pos) delete kc_fixed_pos;
                if (kc_fixed_quat) delete kc_fixed_quat;
                INDICATE_FAILURE
                return FAILURE;
              }
            }
          }
        }
        else
        {
          INDICATE_FAILURE
          return FAILURE;
        }
      }
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    return SUCCESS;
  }

  RigidBodyManipulator* DrakeIKProblem::getDrakeModel()
  {
    return model_;
  }

  EReturn DrakeIKProblem::initDerived(tinyxml2::XMLHandle & handle)
  {
    tinyxml2::XMLHandle tmp_handle = handle.FirstChildElement("URDFPath");
    if (!tmp_handle.ToElement())
    {
      ERROR("URDFPath is required for Drake RigidBodyManipulator");
      return FAILURE;
    }
    std::string urdf_path = tmp_handle.ToElement()->GetText();
    if (model_) delete model_;
    model_ = new RigidBodyManipulator(urdf_path);
    for (int i = 0; i < model_->num_positions; i++)
    {
//      ROS_INFO_STREAM("Joint "<<i<<" "<<model_->getPositionName(i));
      joints_map_[model_->getPositionName(i)] = i;
    }
//    for (int i = 0; i < model_->num_bodies; i++)
//      ROS_INFO_STREAM("Frame "<<i<<" "<<model_->bodies[i]->linkname);

    magic_.l_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
    magic_.l_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624, -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
    magic_.r_foot_pts = Eigen::Matrix3Xd::Zero(3, 4);
    magic_.r_foot_pts << -0.0820, -0.0820, 0.1780, 0.1780, 0.0624, -0.0624, 0.0624, -0.0624, -0.0811, -0.0811, -0.0811, -0.0811;
    return SUCCESS;
  }

  EReturn DrakeIKProblem::buildQuasiStaticConstraint(
      const rapidjson::Value& obj, QuasiStaticConstraint* &ptr)
  {
    bool enabled = false;
    if (!ok(getJSON(obj["enabled"], enabled)))
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    double shrinkFactor = 0;
    if (obj["shrinkFactor"].IsNull()
        || !ok(getJSON(obj["leftFootLinkName"], shrinkFactor)))
    {
      WARNING("ShrinkFactor not defined, set to 0.2");
      shrinkFactor = 0.2;
    }

    ptr = new QuasiStaticConstraint(model_);
    ptr->setActive(enabled);
    ptr->setShrinkFactor(shrinkFactor);
    std::string link_name;
    if (ok(getJSON(obj["leftFootEnabled"], enabled)) && enabled
        && ok(getJSON(obj["leftFootLinkName"], link_name)))
    {
      int idx = model_->findLinkId(link_name);
      ptr->addContact(1, &idx, &magic_.l_foot_pts);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    if (ok(getJSON(obj["rightFootEnabled"], enabled)) && enabled
        && ok(getJSON(obj["rightFootLinkName"], link_name)))
    {
      int idx = model_->findLinkId(link_name);
      ptr->addContact(1, &idx, &magic_.r_foot_pts);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
  }

  EReturn DrakeIKProblem::buildPostureConstraint(const rapidjson::Value& obj,
      PostureConstraint* &ptr)
  {
    std::vector<std::string> joints;
    std::string pose_name;
    Eigen::VectorXd lb, ub;
    if (ok(getJSON(obj["joints"], joints))
        && ok(getJSON(obj["postureName"], pose_name))
        && poses->find(pose_name) != poses->end()
        && (obj["jointsLowerBound"].IsArray() ?
            ok(getJSON(obj["jointsLowerBound"], lb)) :
            ok(getJSON(obj["jointsLowerBound"]["__ndarray__"], lb)))
        && (obj["jointsUpperBound"].IsArray() ?
            ok(getJSON(obj["jointsUpperBound"], ub)) :
            ok(getJSON(obj["jointsUpperBound"]["__ndarray__"], ub))))
    {
      Eigen::VectorXd q_pose = poses->at(pose_name);
      Eigen::VectorXd q(joints.size());
      std::vector<int> joint_idx(joints.size());
      for (int i = 0; i < joints.size(); i++)
      {
        joint_idx[i] = joints_map_.at(joints[i]);
        q(i) = q_pose(joint_idx[i]);
      }
      lb += q;
      ub += q;
      ptr = new PostureConstraint(model_);
      ptr->setJointLimits(joints.size(), joint_idx.data(), lb, ub);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    return SUCCESS;
  }

  EReturn DrakeIKProblem::buildWorldPositionConstraint(
      const rapidjson::Value& obj, WorldPositionConstraint* &ptr)
  {
    Eigen::VectorXd lb, ub;
    Eigen::VectorXd tspan;
    Eigen::VectorXd pointInLink;
    std::string link_name;
    Eigen::VectorXd frame;
    if ((
        obj["lowerBound"].IsArray() ?
            ok(getJSON(obj["lowerBound"], lb)) :
            ok(getJSON(obj["lowerBound"]["__ndarray__"], lb)))
        && (obj["upperBound"].IsArray() ?
            ok(getJSON(obj["upperBound"], ub)) :
            ok(getJSON(obj["upperBound"]["__ndarray__"], ub)))
        && ok(getJSON(obj["tspan"], tspan))
        && ok(getJSON(obj["referenceFrame"]["position"]["__ndarray__"], frame))
        && ok(getJSON(obj["pointInLink"]["__ndarray__"], pointInLink))
        && ok(getJSON(obj["linkName"], link_name)))
    {
      lb += frame;
      ub += frame;
      ptr = new WorldPositionConstraint(model_,
          link_name.compare("pelvis") == 0 ? 1 : model_->findLinkId(link_name),
          pointInLink, lb, ub, tspan);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
  }

  EReturn DrakeIKProblem::buildWorldQuatConstraint(const rapidjson::Value& obj,
      WorldQuatConstraint* &ptr)
  {
    Eigen::VectorXd tspan;
    std::string link_name;
    KDL::Frame frame;
    double tol;
    if (ok(getJSON(obj["tspan"], tspan))
        && ok(getJSON(obj["linkName"], link_name))
        && ok(getJSON(obj["quaternion"], frame))
        && ok(getJSON(obj["angleToleranceInDegrees"], tol)))
    {
      Eigen::Vector4d quat;
      frame.M.GetQuaternion(quat(1), quat(2), quat(3), quat(0));
      ptr = new WorldQuatConstraint(model_,
          link_name.compare("pelvis") == 0 ? 1 : model_->findLinkId(link_name),
          quat, tol, tspan);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }

    return SUCCESS;
  }

  EReturn DrakeIKProblem::buildFixedLinkFromRobotPoseConstraint(
      const rapidjson::Value& obj, WorldPositionConstraint* &pos_ptr,
      WorldQuatConstraint* &quat_ptr)
  {
    std::string link_name;
    Eigen::VectorXd lb, ub;
    Eigen::VectorXd tspan;
    std::string pose_name;
    double tol;
    if (ok(getJSON(obj["linkName"], link_name))
        && ok(getJSON(obj["tspan"], tspan))
        && ok(getJSON(obj["lowerBound"]["__ndarray__"], lb))
        && ok(getJSON(obj["upperBound"]["__ndarray__"], ub))
        && ok(getJSON(obj["poseName"], pose_name))
        && poses->find(pose_name) != poses->end()
        && ok(getJSON(obj["angleToleranceInDegrees"], tol)))
    {
      Eigen::VectorXd q = poses->at(pose_name);
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model_->num_velocities);
      model_->doKinematics(q, v, false, false);
      int idx = model_->findLinkId(link_name);
      Eigen::Vector3d pt = Eigen::Vector3d::Zero();
      Eigen::VectorXd pos = model_->forwardKin(pt, idx, 0, 0, 0).value();
      lb += pos;
      ub += pos;
      pos_ptr = new WorldPositionConstraint(model_, idx,
          Eigen::Vector3d::Zero(), lb, ub, tspan);
      Eigen::Vector4d quat;
      quat << 1, 0, 0, 0;
      quat_ptr = new WorldQuatConstraint(model_, model_->findLinkId(link_name),
          quat, tol, tspan);
    }
    else
    {
      INDICATE_FAILURE
      return FAILURE;
    }
    return SUCCESS;
  }

  EReturn DrakeIKProblem::getBounds(const rapidjson::Value& obj,
      Eigen::Vector2d &lb, Eigen::Vector2d &ub)
  {
    return SUCCESS;
  }
}
