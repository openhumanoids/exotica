//
// Copyright (c) 2018-2020, Wolfgang Merkt, University of Oxford
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

#ifndef EXOTICA_CORE_TASK_MAPS_POINT_TO_PLANE_H_
#define EXOTICA_CORE_TASK_MAPS_POINT_TO_PLANE_H_

#include <exotica_core/task_map.h>

#include <exotica_core_task_maps/point_to_plane_initializer.h>

#include <visualization_msgs/MarkerArray.h>

namespace exotica
{
/**
 * @brief PointToPlane TaskMap: Penalises the z-distance between two frames - or the distance of a point (represented by the Link frame) spanned by the normal represented through the z-axis of a second frame (represented by the Base frame).
 * 
 * This TaskMap returns the signed distance to the plane by default. In an unconstrained optimisation this would correspond to an equality constraint and force the distance to the plane to be minimised. In order to use the TaskMap as an inequality constraint, the parameter 'PositiveOnly' can be set to true. In this case, the TaskMap applies a ReLU-like activation ($x = \max(0, x)$) to the output.
 * 
 */
class PointToPlane : public TaskMap, public Instantiable<PointToPlaneInitializer>
{
public:
    void Instantiate(const PointToPlaneInitializer &init) override;

    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi) override;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian) override;
    void Update(Eigen::VectorXdRefConst q, Eigen::VectorXdRef phi, Eigen::MatrixXdRef jacobian, HessianRef hessian) override;
    int TaskSpaceDim() override;

private:
    void PublishDebug();
    ros::Publisher debug_pub_;
};
}  // namespace exotica

#endif  // EXOTICA_CORE_TASK_MAPS_POINT_TO_PLANE_H_
