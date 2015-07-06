/*
 * OMPLGoalState.cpp
 *
 *  Created on: 17 Jun 2015
 *      Author: yiming
 */

#include "ompl_solver/OMPLGoalState.h"

namespace exotica
{
	OMPLGoalState::OMPLGoalState(const ob::SpaceInformationPtr &si, OMPLProblem_ptr prob) :
			ob::GoalState(si), prob_(prob)
	{
		type_ = ob::GOAL_STATE;
		state_space_ = boost::static_pointer_cast<OMPLStateSpace>(si->getStateSpace());
		setThreshold(std::numeric_limits<double>::epsilon());
	}

	OMPLGoalState::~OMPLGoalState()
	{

	}

	bool OMPLGoalState::isSatisfied(const ompl::base::State *st) const
	{
		return isSatisfied(st, NULL);
	}

	bool OMPLGoalState::isSatisfied(const ompl::base::State *st, double *distance) const
	{
		///	First check if we have a goal configuration
		if (state_)
		{
			double dist = si_->distance(st, state_);
			if (dist > threshold_)
			{
				return false;
			}
		}
		double err = 0.0, tmp;
		bool ret = true, tmpret;
		Eigen::VectorXd q(prob_->getSpaceDim());
		state_space_->copyFromOMPLState(st, q);
		{
			boost::mutex::scoped_lock lock(prob_->getLock());
			prob_->update(q, 0);
			for (TaskTerminationCriterion_ptr goal : prob_->getGoals())
			{
				goal->terminate(tmpret, tmp);
				err += tmp;
				ret = ret && tmpret;
			}
		}
		if (distance)
			*distance = err;
		return ret;
	}

	void OMPLGoalState::sampleGoal(ompl::base::State *st) const
	{
		if (state_)
			si_->copyState(st, state_);
	}

	unsigned int OMPLGoalState::maxSampleCount() const
	{
		if (state_)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}

	void OMPLGoalState::clearGoalState()
	{
		if (state_)
		{
			si_->freeState(state_);
			state_ = NULL;
		}
	}

	const OMPLProblem_ptr OMPLGoalState::getProblem()
	{
		return prob_;
	}
}
