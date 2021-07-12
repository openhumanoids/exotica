//
// Copyright (c) 2018, University of Edinburgh
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <exotica_core/server.h>
#include <exotica_core_task_maps/center_of_mass.h>

REGISTER_TASKMAP_TYPE("CenterOfMass", exotica::CenterOfMass);

namespace exotica
{
void CenterOfMass::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    double M = mass_.sum();
    if (M == 0.0) return;

    KDL::Vector com;
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        com += kinematics[0].Phi(i).p * mass_(i);
        if (debug_)
        {
            com_links_marker_.points[i].x = kinematics[0].Phi(i).p[0];
            com_links_marker_.points[i].y = kinematics[0].Phi(i).p[1];
            com_links_marker_.points[i].z = kinematics[0].Phi(i).p[2];
        }
    }

    com = com / M;
    for (int i = 0; i < dim_; ++i) phi(i) = com[i];

    if (debug_ && Server::IsRos())
    {
        com_marker_.pose.position.x = phi(0);
        com_marker_.pose.position.y = phi(1);
        com_marker_.pose.position.z = phi(2);

        com_marker_.header.stamp = com_links_marker_.header.stamp = ros::Time::now();
        com_links_pub_.publish(com_links_marker_);
        com_pub_.publish(com_marker_);
    }
}

void CenterOfMass::Update(Eigen::VectorXdRefConst x, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != dim_) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != dim_ || jacobian.cols() != x.rows()) ThrowNamed("Wrong size of jacobian! " << x.rows());
    jacobian.setZero();
    KDL::Vector com;

    if (frames_.size() > 0)
    {
        double M = mass_.sum();
        if (M == 0.0) return;
        for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
        {
            com += kinematics[0].Phi(i).p * mass_(i);
            jacobian += mass_(i) / M * kinematics[0].jacobian(i).data.topRows(dim_);
            if (debug_)
            {
                com_links_marker_.points[i].x = kinematics[0].Phi(i).p[0];
                com_links_marker_.points[i].y = kinematics[0].Phi(i).p[1];
                com_links_marker_.points[i].z = kinematics[0].Phi(i).p[2];
            }
        }
        com = com / M;
    }
    else
    {
        if (debug_) com_links_marker_.points.resize(0);
        double M = 0.0;
        for (std::weak_ptr<KinematicElement> welement : scene_->GetKinematicTree().GetTree())
        {
            std::shared_ptr<KinematicElement> element = welement.lock();
            if (element->is_robot_link || element->closest_robot_link.lock())  // Only for robot links and attached objects
            {
                double mass = element->segment.getInertia().getMass();
                if (mass > 0)
                {
                    KDL::Frame cog = KDL::Frame(element->segment.getInertia().getCOG());
                    KDL::Frame com_local = scene_->GetKinematicTree().FK(element, cog, nullptr, KDL::Frame());
                    Eigen::MatrixXd jacobian_com_local = scene_->GetKinematicTree().Jacobian(element, cog, nullptr, KDL::Frame());
                    com += com_local.p * mass;
                    jacobian += jacobian_com_local.topRows(dim_) * mass;
                    M += mass;
                    if (debug_)
                    {
                        geometry_msgs::Point pt;
                        pt.x = com_local.p[0];
                        pt.y = com_local.p[1];
                        pt.z = com_local.p[2];
                        com_links_marker_.points.push_back(pt);
                    }
                }
            }
        }
        if (M == 0.0) return;
        com = com / M;
        jacobian = jacobian / M;
    }
    for (int i = 0; i < dim_; ++i) phi(i) = com[i];

    if (debug_ && Server::IsRos())
    {
        com_marker_.pose.position.x = phi(0);
        com_marker_.pose.position.y = phi(1);
        com_marker_.pose.position.z = phi(2);

        com_marker_.header.stamp = com_links_marker_.header.stamp = ros::Time::now();
        com_links_pub_.publish(com_links_marker_);
        com_pub_.publish(com_marker_);
    }
}

int CenterOfMass::TaskSpaceDim()
{
    return dim_;
}

void CenterOfMass::Initialize()
{
    enable_z_ = parameters_.EnableZ;
    if (enable_z_)
        dim_ = 3;
    else
        dim_ = 2;

    if (frames_.size() > 0)
    {
        // NB: This may break if it's an environment frame (#230)
        HIGHLIGHT_NAMED("CenterOfMass", "!!! WARNING !!! This may be broken and cause a segfault if you update the scene after initialisation as the weak_ptr in GetTreeMap() expire!");
        if (debug_)
            HIGHLIGHT_NAMED("CenterOfMass", "Initialisation with " << frames_.size() << " passed into map.");

        mass_.resize(frames_.size());
        for (std::size_t i = 0; i < frames_.size(); ++i)
        {
            if (frames_[i].frame_B_link_name != "")
            {
                ThrowNamed("Requesting CenterOfMass frame with base other than root! '" << frames_[i].frame_A_link_name << "'");
            }
            frames_[i].frame_A_link_name = scene_->GetKinematicTree().GetTreeMap().at(frames_[i].frame_A_link_name).lock()->segment.getName();
            frames_[i].frame_A_offset.p = scene_->GetKinematicTree().GetTreeMap().at(frames_[i].frame_A_link_name).lock()->segment.getInertia().getCOG();
            mass_(i) = scene_->GetKinematicTree().GetTreeMap().at(frames_[i].frame_A_link_name).lock()->segment.getInertia().getMass();
        }
        if (debug_) HIGHLIGHT_NAMED("CenterOfMass", "Total model mass: " << mass_.sum() << " kg");
    }

    if (Server::IsRos()) InitializeDebug();
}

void CenterOfMass::AssignScene(ScenePtr scene)
{
    scene_ = scene;
    Initialize();
}

void CenterOfMass::InitializeDebug()
{
    com_links_marker_.points.resize(frames_.size());
    com_links_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    com_links_marker_.color.a = .7f;
    com_links_marker_.color.r = 0.5f;
    com_links_marker_.color.g = 0.f;
    com_links_marker_.color.b = 0.f;
    com_links_marker_.scale.x = com_links_marker_.scale.y = com_links_marker_.scale.z = .02;
    com_links_marker_.action = visualization_msgs::Marker::ADD;

    com_marker_.type = visualization_msgs::Marker::SPHERE;
    com_marker_.color.a = 1;
    com_marker_.color.r = 1;
    com_marker_.color.g = 0;
    com_marker_.color.b = 0;
    com_marker_.scale.x = com_marker_.scale.y = com_marker_.scale.z = .05;
    com_marker_.action = visualization_msgs::Marker::ADD;

    com_links_marker_.header.frame_id = com_marker_.header.frame_id =
        "exotica/" + scene_->GetRootFrameName();

    com_links_pub_ = Server::Advertise<visualization_msgs::Marker>(object_name_ + "/com_links_marker", 1, true);
    com_pub_ = Server::Advertise<visualization_msgs::Marker>(object_name_ + "/com_marker", 1, true);
}
}  // namespace exotica
