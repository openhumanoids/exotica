//
// Copyright (c) 2018-2020, University of Edinburgh, University of Oxford
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

#include <exotica_core/task_map.h>
#include <exotica_core/tasks.h>

#include <exotica_core/task_initializer.h>

namespace exotica
{
void Task::Initialize(const std::vector<exotica::Initializer>& inits, PlanningProblemPtr prob, TaskSpaceVector& Phi)
{
    for (const exotica::Initializer& init : inits)
    {
        TaskInitializer task(init);
        auto it = prob->GetTaskMaps().find(task.Task);
        if (it == prob->GetTaskMaps().end()) ThrowPretty("Task map '" << task.Task << "' has not been defined!");
        task_maps[task.Task] = it->second;
        tasks.push_back(it->second);
        task_initializers_.push_back(task);
    }
    num_tasks = tasks.size();
    length_Phi = 0;
    length_jacobian = 0;
    Phi.map.resize(0);
    indexing.resize(tasks.size());
    for (int i = 0; i < num_tasks; ++i)
    {
        indexing[i].id = i;
        indexing[i].start = length_Phi;
        indexing[i].length = tasks[i]->length;
        indexing[i].start_jacobian = length_jacobian;
        indexing[i].length_jacobian = tasks[i]->length_jacobian;

        AppendVector(Phi.map, TaskVectorEntry::reindex(tasks[i]->GetLieGroupIndices(), tasks[i]->start, indexing[i].start));
        length_Phi += tasks[i]->length;
        length_jacobian += tasks[i]->length_jacobian;
    }
    Phi.SetZero(length_Phi);
}

void EndPoseTask::Initialize(const std::vector<exotica::Initializer>& inits, PlanningProblemPtr prob, TaskSpaceVector& unused)
{
    Task::Initialize(inits, prob, Phi);
    y = Phi;
    y.SetZero(length_Phi);
    rho = Eigen::VectorXd::Ones(num_tasks);
    if (prob->GetFlags() & KIN_J) jacobian = Eigen::MatrixXd(length_jacobian, prob->N);
    if (prob->GetFlags() & KIN_H) hessian.setConstant(length_jacobian, Eigen::MatrixXd::Zero(prob->N, prob->N));
    S = Eigen::MatrixXd::Identity(length_jacobian, length_jacobian);
    ydiff = Eigen::VectorXd::Zero(length_jacobian);

    for (int i = 0; i < num_tasks; ++i)
    {
        TaskInitializer task(inits[i]);
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == tasks[i]->length)
        {
            y.data.segment(indexing[i].start, indexing[i].length) = task.Goal;
        }
        else
        {
            ThrowPretty("Invalid task goal size! Expecting " << tasks[i]->length << " got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            rho(i) = 1.0;
        }
        else if (task.Rho.rows() == 1)
        {
            rho(i) = task.Rho(0);
        }
        else
        {
            ThrowPretty("Invalid task rho size! Expecting 1 got " << task.Rho.rows());
        }
    }
}

void EndPoseTask::UpdateS()
{
    for (const TaskIndexing& task : indexing)
    {
        for (int i = 0; i < task.length_jacobian; ++i)
        {
            S(i + task.start_jacobian, i + task.start_jacobian) = rho(task.id);
        }
        if (rho(task.id) != 0.0) tasks[task.id]->is_used = true;
    }
}

void EndPoseTask::Update(const TaskSpaceVector& big_Phi, Eigen::MatrixXdRefConst big_jacobian, HessianRefConst big_hessian)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi.data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        jacobian.middleRows(task.start_jacobian, task.length_jacobian) = big_jacobian.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        hessian.segment(task.start_jacobian, task.length_jacobian) = big_hessian.segment(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff = Phi - y;
}

void EndPoseTask::Update(const TaskSpaceVector& big_Phi, Eigen::MatrixXdRefConst big_jacobian)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi.data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        jacobian.middleRows(task.start_jacobian, task.length_jacobian) = big_jacobian.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff = Phi - y;
}

void EndPoseTask::Update(const TaskSpaceVector& big_Phi)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi.data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
    }
    ydiff = Phi - y;
}

void EndPoseTask::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != indexing[i].length) ThrowPretty("Expected length of " << indexing[i].length << " and got " << goal.rows());
            y.data.segment(indexing[i].start, indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task Map '" << task_name << "' does not exist.");
}

void EndPoseTask::SetRho(const std::string& task_name, const double rho_in)
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            rho(indexing[i].id) = rho_in;
            UpdateS();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task Map '" << task_name << "' does not exist.");
}

Eigen::VectorXd EndPoseTask::GetGoal(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return y.data.segment(indexing[i].start, indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task Map '" << task_name << "' does not exist.");
}

double EndPoseTask::GetRho(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return rho(indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task Map '" << task_name << "' does not exist.");
}

Eigen::MatrixXd EndPoseTask::GetS(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            // We are interested in the square matrix of dimension length_jacobian
            return S.block(indexing[i].start_jacobian, indexing[i].start_jacobian, indexing[i].length_jacobian, indexing[i].length_jacobian);
        }
    }
    ThrowPretty("Cannot get S. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd EndPoseTask::GetTaskError(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return ydiff.segment(indexing[i].start_jacobian, indexing[i].length_jacobian);
        }
    }
    ThrowPretty("Cannot get task error. Task map '" << task_name << "' does not exist.");
}

Eigen::MatrixXd EndPoseTask::GetTaskJacobian(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return jacobian.middleRows(indexing[i].start_jacobian, indexing[i].length_jacobian);
        }
    }
    ThrowPretty("Cannot get task Jacobian. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedTask::Initialize(const std::vector<exotica::Initializer>& inits, PlanningProblemPtr prob, TaskSpaceVector& Phi)
{
    Task::Initialize(inits, prob, Phi);
    Phi.SetZero(length_Phi);
}

void TimeIndexedTask::UpdateS()
{
    for (int t = 0; t < T; ++t)
    {
        for (const TaskIndexing& task : indexing)
        {
            for (int i = 0; i < task.length_jacobian; ++i)
            {
                S[t](i + task.start_jacobian, i + task.start_jacobian) = rho[t](task.id);
            }
            if (rho[t](task.id) != 0.0) tasks[task.id]->is_used = true;
        }
    }
}

void TimeIndexedTask::Update(const TaskSpaceVector& big_Phi,
                             Eigen::MatrixXdRefConst big_dPhi_dx,
                             Eigen::MatrixXdRefConst big_dPhi_du,
                             HessianRefConst big_ddPhi_ddx,
                             HessianRefConst big_ddPhi_ddu,
                             HessianRefConst big_ddPhi_dxdu,
                             int t)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi[t].data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        dPhi_dx[t].middleRows(task.start_jacobian, task.length_jacobian) = big_dPhi_dx.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        dPhi_du[t].middleRows(task.start_jacobian, task.length_jacobian) = big_dPhi_du.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        ddPhi_ddx[t].segment(task.start_jacobian, task.length_jacobian) = big_ddPhi_ddx.segment(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        ddPhi_ddu[t].segment(task.start_jacobian, task.length_jacobian) = big_ddPhi_ddu.segment(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        ddPhi_dxdu[t].segment(task.start_jacobian, task.length_jacobian) = big_ddPhi_dxdu.segment(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff[t] = Phi[t] - y[t];
}

void TimeIndexedTask::Update(const TaskSpaceVector& big_Phi,
                             Eigen::MatrixXdRefConst big_dPhi_dx,
                             Eigen::MatrixXdRefConst big_dPhi_du,
                             int t)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi[t].data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        dPhi_dx[t].middleRows(task.start_jacobian, task.length_jacobian) = big_dPhi_dx.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        dPhi_du[t].middleRows(task.start_jacobian, task.length_jacobian) = big_dPhi_du.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff[t] = Phi[t] - y[t];
}

void TimeIndexedTask::Update(const TaskSpaceVector& big_Phi, Eigen::MatrixXdRefConst big_jacobian, HessianRefConst big_hessian, int t)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi[t].data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        jacobian[t].middleRows(task.start_jacobian, task.length_jacobian) = big_jacobian.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
        hessian[t].segment(task.start_jacobian, task.length_jacobian) = big_hessian.segment(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff[t] = Phi[t] - y[t];
}

void TimeIndexedTask::Update(const TaskSpaceVector& big_Phi, Eigen::MatrixXdRefConst big_jacobian, int t)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi[t].data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
        jacobian[t].middleRows(task.start_jacobian, task.length_jacobian) = big_jacobian.middleRows(tasks[task.id]->start_jacobian, tasks[task.id]->length_jacobian);
    }
    ydiff[t] = Phi[t] - y[t];
}

void TimeIndexedTask::Update(const TaskSpaceVector& big_Phi, int t)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi[t].data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
    }
    ydiff[t] = Phi[t] - y[t];
}

inline void TimeIndexedTask::ValidateTimeIndex(int& t_in) const
{
    if (t_in >= T || t_in < -1)
    {
        ThrowPretty("Requested t=" << t_in << " out of range, needs to be 0 =< t < " << T);
    }
    else if (t_in == -1)
    {
        t_in = (T - 1);
    }
}

void TimeIndexedTask::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal, int t)
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != indexing[i].length) ThrowPretty("Expected length of " << indexing[i].length << " and got " << goal.rows());
            y[t].data.segment(indexing[i].start, indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedTask::GetGoal(const std::string& task_name, int t) const
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return y[t].data.segment(indexing[i].start, indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedTask::SetRho(const std::string& task_name, const double rho_in, int t)
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            rho[t](indexing[i].id) = rho_in;
            UpdateS();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task map '" << task_name << "' does not exist.");
}

double TimeIndexedTask::GetRho(const std::string& task_name, int t) const
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return rho[t](indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task map '" << task_name << "' does not exist.");
}

Eigen::VectorXd TimeIndexedTask::GetTaskError(const std::string& task_name, int t) const
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return ydiff[t].segment(indexing[i].start_jacobian, indexing[i].length_jacobian);
        }
    }
    ThrowPretty("Cannot get ydiff. Task map '" << task_name << "' does not exist.");
}

Eigen::MatrixXd TimeIndexedTask::GetS(const std::string& task_name, int t) const
{
    ValidateTimeIndex(t);
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            // We are interested in the square matrix of dimension length_jacobian
            return S[t].block(indexing[i].start_jacobian, indexing[i].start_jacobian, indexing[i].length_jacobian, indexing[i].length_jacobian);
        }
    }
    ThrowPretty("Cannot get S. Task map '" << task_name << "' does not exist.");
}

void TimeIndexedTask::ReinitializeVariables(int _T, PlanningProblemPtr _prob, const TaskSpaceVector& _Phi)
{
    T = _T;
    Phi.assign(_T, _Phi);
    y = Phi;
    rho.assign(T, Eigen::VectorXd::Ones(num_tasks));

    if (_prob->GetFlags() & KIN_J)
    {
        jacobian.assign(T, Eigen::MatrixXd(length_jacobian, _prob->N));
        dPhi_dx.assign(T, Eigen::MatrixXd(length_jacobian, _prob->GetScene()->get_num_state_derivative()));
        dPhi_du.assign(T, Eigen::MatrixXd(length_jacobian, _prob->GetScene()->get_num_controls()));
    }
    if (_prob->GetFlags() & KIN_H)
    {
        hessian.assign(T, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(_prob->N, _prob->N)));
        ddPhi_ddx.assign(T, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(_prob->GetScene()->get_num_state_derivative(), _prob->GetScene()->get_num_state_derivative())));
        ddPhi_ddu.assign(T, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(_prob->GetScene()->get_num_controls(), _prob->GetScene()->get_num_controls())));
        ddPhi_dxdu.assign(T, Hessian::Constant(length_jacobian, Eigen::MatrixXd::Zero(_prob->GetScene()->get_num_state_derivative(), _prob->GetScene()->get_num_controls())));
    }
    S.assign(T, Eigen::MatrixXd::Identity(length_jacobian, length_jacobian));
    ydiff.assign(T, Eigen::VectorXd::Zero(length_jacobian));

    if (num_tasks != task_initializers_.size()) ThrowPretty("Number of tasks does not match internal number of tasks!");
    for (int i = 0; i < num_tasks; ++i)
    {
        TaskInitializer& task = task_initializers_[i];
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == tasks[i]->length * T)
        {
            for (int t = 0; t < T; ++t)
            {
                y[t].data.segment(indexing[i].start, indexing[i].length) = task.Goal.segment(t * tasks[i]->length, tasks[i]->length);
            }
        }
        else if (task.Goal.rows() == tasks[i]->length)
        {
            for (int t = 0; t < T; ++t)
            {
                y[t].data.segment(indexing[i].start, indexing[i].length) = task.Goal;
            }
        }
        else
        {
            ThrowPretty("Invalid task goal size! Expecting " << tasks[i]->length * T << ", " << tasks[i]->length << ", or 1 and got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            // Keep ones
        }
        else if (task.Rho.rows() == T)
        {
            for (int t = 0; t < T; ++t)
            {
                rho[t](i) = task.Rho(t);
            }
        }
        else if (task.Rho.rows() == 1)
        {
            for (int t = 0; t < T; ++t)
            {
                rho[t](i) = task.Rho(0);
            }
        }
        else
        {
            ThrowPretty("Invalid task rho size! Expecting " << T << " (or 1) and got " << task.Rho.rows());
        }
    }
}

void SamplingTask::Initialize(const std::vector<exotica::Initializer>& inits, PlanningProblemPtr prob, TaskSpaceVector& unused)
{
    Task::Initialize(inits, prob, Phi);
    y = Phi;
    y.SetZero(length_Phi);
    rho = Eigen::VectorXd::Ones(num_tasks);
    S = Eigen::MatrixXd::Identity(length_jacobian, length_jacobian);
    ydiff = Eigen::VectorXd::Zero(length_jacobian);

    for (int i = 0; i < num_tasks; ++i)
    {
        TaskInitializer task(inits[i]);
        if (task.Goal.rows() == 0)
        {
            // Keep zero goal
        }
        else if (task.Goal.rows() == tasks[i]->length)
        {
            y.data.segment(indexing[i].start, indexing[i].length) = task.Goal;
        }
        else
        {
            ThrowPretty("Invalid task goal size! Expecting " << tasks[i]->length << " got " << task.Goal.rows());
        }
        if (task.Rho.rows() == 0)
        {
            rho(i) = 1.0;
        }
        else if (task.Rho.rows() == 1)
        {
            rho(i) = task.Rho(0);
        }
        else
        {
            ThrowPretty("Invalid task rho size! Expecting 1 got " << task.Rho.rows());
        }
    }
}

void SamplingTask::UpdateS()
{
    for (const TaskIndexing& task : indexing)
    {
        for (int i = 0; i < task.length_jacobian; ++i)
        {
            S(i + task.start_jacobian, i + task.start_jacobian) = rho(task.id);
        }
        if (rho(task.id) != 0.0) tasks[task.id]->is_used = true;
    }
}

void SamplingTask::Update(const TaskSpaceVector& big_Phi)
{
    for (const TaskIndexing& task : indexing)
    {
        Phi.data.segment(task.start, task.length) = big_Phi.data.segment(tasks[task.id]->start, tasks[task.id]->length);
    }
    ydiff = Phi - y;

    for (unsigned int i = 0; i < ydiff.size(); ++i)
        if (std::abs(ydiff[i]) < tolerance) ydiff[i] = 0.0;
}

void SamplingTask::SetGoal(const std::string& task_name, Eigen::VectorXdRefConst goal)
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            if (goal.rows() != indexing[i].length) ThrowPretty("Expected length of " << indexing[i].length << " and got " << goal.rows());
            y.data.segment(indexing[i].start, indexing[i].length) = goal;
            return;
        }
    }
    ThrowPretty("Cannot set Goal. Task Map '" << task_name << "' does not exist.");
}

void SamplingTask::SetRho(const std::string& task_name, const double rho_in)
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            rho(indexing[i].id) = rho_in;
            UpdateS();
            return;
        }
    }
    ThrowPretty("Cannot set rho. Task Map '" << task_name << "' does not exist.");
}

Eigen::VectorXd SamplingTask::GetGoal(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return y.data.segment(indexing[i].start, indexing[i].length);
        }
    }
    ThrowPretty("Cannot get Goal. Task Map '" << task_name << "' does not exist.");
}

double SamplingTask::GetRho(const std::string& task_name) const
{
    for (size_t i = 0; i < indexing.size(); ++i)
    {
        if (tasks[i]->GetObjectName() == task_name)
        {
            return rho(indexing[i].id);
        }
    }
    ThrowPretty("Cannot get rho. Task Map '" << task_name << "' does not exist.");
}
}  // namespace exotica
