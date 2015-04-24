/*
 * frrt.cpp
 *
 *  Created on: 22 Apr 2015
 *      Author: yiming
 */

#include "frrt/FRRT.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/goals/GoalSampleableRegion.h"

namespace ompl
{
	namespace geometric
	{
		FRRT::FRRT(const base::SpaceInformationPtr &si) :
				base::Planner(si, "FRRT")
		{
			specs_.approximateSolutions = true;
			specs_.directed = true;
			goalBias_ = 0.05;
			maxDist_ = 0.0;
			lastGoalMotion_ = NULL;
			Planner::declareParam<double>("range", this, &FRRT::setRange, &FRRT::getRange, "0.:1.:10000.");
			Planner::declareParam<double>("goal_bias", this, &FRRT::setGoalBias, &FRRT::getGoalBias, "0.:.05:1.");

			global_try_ = 0;
			global_succeeded_ = 0;
			local_try_ = 0;
			local_succeeded_ = 0;
			normal_try_ = 0;
			normal_succeeded_ = 0;
		}

		FRRT::~FRRT()
		{
			freeMemory();
		}

		void FRRT::clear()
		{
			Planner::clear();
			sampler_.reset();
			freeMemory();
			if (nn_)
				nn_->clear();
			lastGoalMotion_ = NULL;
		}

		void FRRT::setup()
		{
			Planner::setup();
			tools::SelfConfig sc(si_, getName());
			sc.configurePlannerRange(maxDist_);

			if (!nn_)
				nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
			nn_->setDistanceFunction(boost::bind(&FRRT::distanceFunction, this, _1, _2));
		}

		void FRRT::freeMemory()
		{
			std::vector<Motion*> motions;
			nn_->list(motions);
			for (unsigned int i = 0; i < motions.size(); ++i)
			{
				if (motions[i]->state)
					si_->freeState(motions[i]->state);
				delete motions[i];
			}
		}

		base::PlannerStatus FRRT::solve(const base::PlannerTerminationCondition &ptc)
		{
			if (!local_solver_)
			{
				INDICATE_FAILURE
				return base::PlannerStatus::CRASH;
			}
			checkValidity();
			base::Goal *goal = pdef_->getGoal().get();
			base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
			while (const base::State *st = pis_.nextStart())
			{
				Motion *motion = new Motion(si_);
				si_->copyState(motion->state, st);
				nn_->add(motion);
			}

			if (nn_->size() == 0)
			{
				OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
				INDICATE_FAILURE
				return base::PlannerStatus::INVALID_START;
			}
			if (!sampler_)
				sampler_ = si_->allocStateSampler();
			OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

			Motion *solution = NULL;
			Motion *approxsol = NULL;
			double approxdif = std::numeric_limits<double>::infinity();
			Motion *rmotion = new Motion(si_);
			base::State *rstate = rmotion->state;
			base::State *xstate = si_->allocState();
			Motion *last_motion = rmotion;
			while (ptc == false)
			{
				bool valid_state = false;
				Motion *motion = new Motion(si_);

				if (true)
				{
					/* set the local to global goal */
					global_try_++;
					global_task_->setRho(1, 0);
					local_task_->setRho(0, 0);
					if (localSolve(last_motion, motion))
					{
						/* This is where the local planner goes */
						valid_state = true;
						global_succeeded_++;
					}
					else
					{
						;
					}

				}
				else
				{
					ERROR("Can not sample goal");
				}
				if (!valid_state)
				{
					sampler_->sampleUniform(rstate);
					/* find closest state in the tree */
					Motion *nmotion = nn_->nearest(rmotion);
					base::State *dstate = rstate;
					si_->copyState(motion->state, dstate);

					normal_try_++;
					if (si_->checkMotion(nmotion->state, dstate))
					{
						motion->parent = nmotion;
						valid_state = true;
						normal_succeeded_++;
					}
					else
					{

						local_try_++;
						/* set the local to local goal */
						Eigen::VectorXd eigen_g((int) si_->getStateDimension());
						memcpy(eigen_g.data(), rstate->as<
								ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
								* eigen_g.rows());

						global_task_->setRho(0, 0);
						local_task_->setRho(1, 0);
						local_task_->setGoal(eigen_g, 0);
						if (localSolve(nmotion, motion))
						{
							/* This is where the local planner goes */
							local_succeeded_++;
							valid_state = true;
						}
						else
						{
							; //ERROR("Local planning failed");
						}
					}
				}
				if (valid_state)
				{
					nn_->add(motion);
					double dist = 0.0;
					bool sat = goal->isSatisfied(motion->state, &dist);
					if (sat)
					{
						approxdif = dist;
						solution = motion;
						break;
					}
					if (dist < approxdif)
					{
						approxdif = dist;
						approxsol = motion;
					}
				}
				last_motion = motion;
			}
			bool solved = false;
			bool approximate = false;
			if (solution == NULL)
			{
				solution = approxsol;
				approximate = true;
			}
			if (solution != NULL)
			{
				lastGoalMotion_ = solution;
				/* construct the solution path */
				std::vector<Motion*> mpath;
				while (solution != NULL)
				{
					if (solution->internal_path != nullptr)
					{
						for (int i = solution->internal_path->rows() - 1; i > 0; i--)
						{
							Motion *local_motion = new Motion(si_);
							Eigen::VectorXd tmp = solution->internal_path->row(i);
							memcpy(local_motion->state->as<
									ompl::base::RealVectorStateSpace::StateType>()->values, tmp.data(), sizeof(double)
									* (int) si_->getStateDimension());
							mpath.push_back(local_motion);
						}
					}
					else
						mpath.push_back(solution);
					solution = solution->parent;
				}
				PathGeometric *path = new PathGeometric(si_);
				for (int i = mpath.size() - 1; i >= 0; --i)
					path->append(mpath[i]->state);
				pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif, getName());
				solved = true;
			}

			si_->freeState(xstate);
			if (rmotion->state)
				si_->freeState(rmotion->state);
			delete rmotion;
			OMPL_INFORM("%s: Created %d states", getName().c_str(), nn_->size());
			OMPL_INFORM("%s: Global succeeded/try %d / %d", getName().c_str(), global_succeeded_, global_try_);
			OMPL_INFORM("%s: Local succeeded/try %d / %d", getName().c_str(), local_succeeded_, local_try_);
			OMPL_INFORM("%s: Normal succeeded/try %d / %d", getName().c_str(), normal_succeeded_, normal_try_);

			return base::PlannerStatus(solved, approximate);
		}

		void FRRT::getPlannerData(base::PlannerData &data) const
		{
			Planner::getPlannerData(data);
			std::vector<Motion*> motions;
			if (nn_)
				nn_->list(motions);
			if (lastGoalMotion_)
				data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));
			for (unsigned int i = 0; i < motions.size(); ++i)
			{
				if (motions[i]->parent == NULL)
					data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
				else
					data.addEdge(base::PlannerDataVertex(motions[i]->parent->state), base::PlannerDataVertex(motions[i]->state));
			}
		}

		bool FRRT::setUpLocalPlanner(const std::string & xml_file, const exotica::Scene_ptr & scene)
		{
			exotica::Initialiser ini;
			exotica::Server_ptr ser;
			exotica::PlanningProblem_ptr prob;
			exotica::MotionSolver_ptr sol;
			if (!ok(ini.initialise(xml_file, ser, sol, prob)))
			{
				INDICATE_FAILURE
				return false;
			}
			if (sol->type().compare("exotica::IKsolver") == 0)
			{
				local_solver_ = boost::static_pointer_cast<exotica::IKsolver>(sol);
			}
			else
			{
				INDICATE_FAILURE
				return false;
			}
			if (!ok(local_solver_->specifyProblem(prob)))
			{
				INDICATE_FAILURE
				return false;
			}
			prob->setScene(scene->getPlanningScene());

			base::Goal *goal = pdef_->getGoal().get();
			if (!goal)
			{
				INDICATE_FAILURE
				return false;
			}
			if (prob->getTaskDefinitions().find("LocalTask") == prob->getTaskDefinitions().end())
			{
				ERROR("Missing XML tag of 'LocalTask'");
				return false;
			}
			if (prob->getTaskDefinitions().find("GlobalTask") == prob->getTaskDefinitions().end())
			{
				ERROR("Missing XML tag of 'GlobalTask'");
				return false;
			}
			local_task_ =
					boost::static_pointer_cast<exotica::TaskSqrError>(prob->getTaskDefinitions().at("LocalTask"));
			global_task_ =
					boost::static_pointer_cast<exotica::TaskSqrError>(prob->getTaskDefinitions().at("GlobalTask"));
			return true;
		}

		bool FRRT::localSolve(Motion *sm, Motion *gm)
		{
			int dim = (int) si_->getStateDimension();
			Eigen::VectorXd qs(dim), qg(dim);
			memcpy(qs.data(), sm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values, sizeof(double)
					* qs.rows());

			Eigen::MatrixXd local_path;
			if (ok(local_solver_->SolveWithFullSolution(qs, local_path)))
			{
				/* Local planner succeeded */
				gm->internal_path.reset(new Eigen::MatrixXd(local_path));
				gm->parent = sm;
				qg = local_path.row(local_path.rows() - 1).transpose();
				memcpy(gm->state->as<ompl::base::RealVectorStateSpace::StateType>()->values, qg.data(), sizeof(double)
						* qs.rows());
				return true;
			}
			return false;
		}
	}	//	Namespace geometric
}	//	Namespace ompl

