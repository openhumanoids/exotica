//
// Copyright (c) 2018-2021, Wolfgang Merkt, University of Oxford
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
#include <exotica_core/tools/conversions.h>
#include <exotica_core_task_maps/point_to_plane.h>
#include <kdl_conversions/kdl_msg.h>

REGISTER_TASKMAP_TYPE("PointToPlane", exotica::PointToPlane);

namespace exotica
{
void PointToPlane::Instantiate(const PointToPlaneInitializer& init)
{
    Instantiable<PointToPlaneInitializer>::Instantiate(init);  // To assign parameters_ = init

    if (debug_ && Server::IsRos())
    {
        debug_pub_ = Server::Advertise<visualization_msgs::MarkerArray>(object_name_ + "/planes", 1, true);
    }

    if (debug_)
    {
        for (std::size_t i = 0; i < frames_.size(); ++i)
        {
            HIGHLIGHT_NAMED(object_name_, "#" << i << " Plane: " << frames_[i].frame_B_link_name << " " << GetFrameAsVector(frames_[i].frame_B_offset).transpose() << " - Query Point: " << frames_[i].frame_A_link_name << " (" << GetFrameAsVector(frames_[i].frame_A_offset).transpose() << ")");
        }
    }
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of phi!");

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        if (!parameters_.PositiveOnly)
        {
            phi(i) = kinematics[0].Phi(i).p.data[2];
        }
        else
        {
            phi(i) = std::max(0.0, kinematics[0].Phi(i).p.data[2]);
        }
    }

    if (debug_ && Server::IsRos()) PublishDebug();
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        if (!parameters_.PositiveOnly)
        {
            phi(i) = kinematics[0].Phi(i).p.data[2];
            jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
        }
        else
        {
            phi(i) = kinematics[0].Phi(i).p.data[2];

            // For -inf to 0.0, the constraint is inactive, i.e. we will return 0:
            if (phi(i) <= 0.0)
            {
                phi(i) = 0.0;
            }
            // For phi > 0.0, we will return the distance and Jacobian
            else
            {
                jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
            }

            if (debug_)
            {
                HIGHLIGHT_NAMED(object_name_, "PositiveOnly: " << kinematics[0].Phi(i).p.data[2] << " --> " << phi(i));
            }
        }
    }

    if (debug_ && Server::IsRos()) PublishDebug();
}

void PointToPlane::Update(Eigen::VectorXdRefConst /*q*/, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian)
{
    if (phi.rows() != kinematics[0].Phi.rows()) ThrowNamed("Wrong size of Phi!");
    if (jacobian.rows() != kinematics[0].jacobian.rows() || jacobian.cols() != kinematics[0].jacobian(0).data.cols()) ThrowNamed("Wrong size of jacobian! " << kinematics[0].jacobian(0).data.cols());

    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        if (!parameters_.PositiveOnly)
        {
            phi(i) = kinematics[0].Phi(i).p.data[2];
            jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
            hessian(i).block(0, 0, jacobian.cols(), jacobian.cols()) = kinematics[0].hessian[i](2);
        }
        else
        {
            phi(i) = std::max(0.0, kinematics[0].Phi(i).p.data[2]);
            if (phi(i) > 0.0)
            {
                jacobian.row(i) = kinematics[0].jacobian[i].data.middleRows<1>(2);
                hessian(i).block(0, 0, jacobian.cols(), jacobian.cols()) = kinematics[0].hessian[i](2);
            }
        }
    }
}

int PointToPlane::TaskSpaceDim()
{
    return kinematics[0].Phi.rows();
}

void PointToPlane::PublishDebug()
{
    visualization_msgs::MarkerArray msg;

    msg.markers.reserve(3 * kinematics[0].Phi.rows());
    for (int i = 0; i < kinematics[0].Phi.rows(); ++i)
    {
        // Plane
        visualization_msgs::Marker plane;
        plane.header.frame_id = frames_[i].frame_B_link_name == "" ? "exotica/" + scene_->GetRootFrameName() : frames_[i].frame_B_link_name;
        plane.ns = frames_[i].frame_A_link_name;
        plane.id = i;
        plane.type = visualization_msgs::Marker::CUBE;
        plane.action = visualization_msgs::Marker::ADD;
        plane.frame_locked = true;
        plane.color.g = 1.0f;
        plane.color.a = 0.8f;
        tf::poseKDLToMsg(frames_[i].frame_B_offset, plane.pose);
        plane.scale.x = 10.f;
        plane.scale.y = 10.f;
        plane.scale.z = 0.01f;
        plane.pose.position.z -= (plane.scale.z / 2);
        msg.markers.push_back(plane);

        // Vector of Normal
        visualization_msgs::Marker normal_vector;
        normal_vector.header.frame_id = frames_[i].frame_B_link_name == "" ? "exotica/" + scene_->GetRootFrameName() : frames_[i].frame_B_link_name;
        normal_vector.ns = frames_[i].frame_A_link_name + " - Plane Normal";
        normal_vector.id = 1000 + i;
        normal_vector.type = visualization_msgs::Marker::ARROW;
        normal_vector.action = visualization_msgs::Marker::ADD;
        normal_vector.frame_locked = true;
        normal_vector.color.r = 1.0f;
        normal_vector.color.a = 0.8f;
        // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis.
        // We want it to point around the +Z-axis:
        KDL::Frame rotate_x_up_to_z_up = KDL::Frame::Identity();
        rotate_x_up_to_z_up.M = KDL::Rotation::RotY(-M_PI_2);
        tf::poseKDLToMsg(rotate_x_up_to_z_up * frames_[i].frame_B_offset, normal_vector.pose);
        normal_vector.scale.x = 0.25f;
        normal_vector.scale.y = 0.05f;
        normal_vector.scale.z = 0.05f;
        msg.markers.push_back(normal_vector);

        // Query Point
        visualization_msgs::Marker query_point;
        query_point.header.frame_id = frames_[i].frame_A_link_name == "" ? "exotica/" + scene_->GetRootFrameName() : "exotica/" + frames_[i].frame_A_link_name;
        query_point.ns = frames_[i].frame_A_link_name + " - Query Point";
        query_point.id = 2000 + i;
        query_point.type = visualization_msgs::Marker::SPHERE;
        query_point.action = visualization_msgs::Marker::ADD;
        query_point.frame_locked = true;
        query_point.color.b = 1.0f;
        query_point.color.a = 0.8f;
        tf::poseKDLToMsg(frames_[i].frame_A_offset, query_point.pose);
        query_point.scale.x = 0.05f;
        query_point.scale.y = 0.05f;
        query_point.scale.z = 0.05f;
        msg.markers.push_back(query_point);
    }

    debug_pub_.publish(msg);
}
}  // namespace exotica
