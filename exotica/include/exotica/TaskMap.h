/***********************************************************************\
|    TaskMap defines the forward mapping for tasks (basically the       |
 |   phi-function definition and its derivatives). It is an abstract     |
 |   class. NOT Entirely Thread-safe.                                    |
 |                                                                       |
 |           Developer: Michael Camilleri (mcamnadur@gmail.com)          |
 |                    Last Edited: 28 - March - 2014                     |
 \***********************************************************************/

#ifndef EXOTICA_TASK_MAP_H
#define EXOTICA_TASK_MAP_H

#include "exotica/Object.h"       //!< The EXOTica base class
#include "exotica/Factory.h"      //!< The Factory template
#include "exotica/Test.h"         //!< The Test factory template
#include "exotica/Server.h"
#include "exotica/Scene.h"

#include <Eigen/Dense>            //!< Generally dense manipulations should be enough
#include <boost/thread/mutex.hpp> //!< The boost thread-library for synchronisation
#include <string>

/**
 * \brief Convenience registrar for the TaskMap Type
 */
#define REGISTER_TASKMAP_TYPE(TYPE, DERIV) EXOTICA_REGISTER(std::string, exotica::TaskMap, TYPE, DERIV)

#define PHI (*(phi_.at(t)))
#define JAC (*(jac_.at(t)))
#define EFFPHI (*eff_phi_)
#define EFFJAC (*eff_jac_)

namespace exotica
{
	class TaskMap: public Object
	{
		public:
			/**
			 * \brief Default Constructor
			 */
			TaskMap();
			virtual ~TaskMap()
			{
			}


			/**
			 * \brief Initialiser (from XML): mainly resolves the KinematicScene pointer
			 * @pre             The Kinematic Scenes must already be initialised
			 * @post            If the xml-element contains a 'kscene' tag, then it will attempt to bind the map to that kinematic scene. Otherwise, scene_ will be set to a nullptr.
			 * @post            Will call the initDerived() function if everything is successful.
			 * @param handle    The handle to the XML-element describing the task map
			 * @param scene_ptr Map of kinematic scenes (Optional: defaulted)
			 * @return          Result of initDerived() if initialisation successful,
			 *                  \n PAR_ERR if could not bind scene information.
			 */
			EReturn initBase(tinyxml2::XMLHandle & handle, Server_ptr & server,
					const Scene_map & scene_ptr = Scene_map());

			/**
			 * \brief Updates the output functions (phi and jacobian): PURE VIRTUAL
			 * \details The Function should:
			 *          \n call invalidate() before starting task-specific execution
			 *          \n lock the scene_ pointer (using the scene_lock_ mutex)
			 *          \n check that everything it needs (including possibly the scene pointer) is valid
			 * @post      Should store the results using setY() and setYDot() if successful
			 * @param  x  The State-space vector for the robot
			 * @return    Should indicate success/otherwise using the Exotica error types
			 */
            virtual EReturn update(Eigen::VectorXdRefConst x, const int t) = 0;

            /**
             * @brief registerPhi Registers a memory location for the output of phi at time t
             * @param y Reference to memory location to be registered
             * @param t Time step
             * @return Indication of success
             */
            EReturn registerPhi(Eigen::VectorXdRef_ptr y, int t);

            /**
             * @brief registerJacobian egisters a memory location for the output of Jacobian at time t
             * @param J Reference to memory location to be registered
             * @param t Time step
             * @return Indication of success
             */
            EReturn registerJacobian(Eigen::MatrixXdRef_ptr J, int t);

			/**
			 * \brief Indicator of the Task-Dimension size: PURE VIRTUAL
			 * @param task_dim  The dimensionality of the Task space, or -1 if dynamic...
			 * @return          SUCCESS if ok, MMB_NIN if called on an uninitialised object
			 */
			virtual EReturn taskSpaceDim(int & task_dim) = 0;

			/**
			 * @brief setTimeSteps Sets number of timesteps for tasks that require to keep track of task space coordinates over time (ignored in other tasks)
			 * @param T Number of time steps (this should be set by the planning problem)
			 * @return Returns success.
			 */
			virtual EReturn setTimeSteps(const int T);

            bool isRegistered(int t);

            Scene_ptr getScene();

            bool updateJacobian_;
		protected:

			/**
			 * \brief Initialises members of the derived type: PURE_VIRTUAL
			 * @param handle  The handle to the XML-element describing the task
			 * @return        Should indicate success/failure
			 */
			virtual EReturn initDerived(tinyxml2::XMLHandle & handle) = 0;

			/**
			 * Member Variables
			 */
			Scene_ptr scene_;  //!< The Scene object (smart-pointer):
			boost::mutex scene_lock_;  //!< Synchronisation for the scene object
			Server_ptr server_; //!< Pointer to EXOTica parameter server;
			/**
			 * \brief Private data members for information hiding and thread-safety
			 */
            std::vector<Eigen::VectorXdRef_ptr> phi_;       //!< The Task-space co-ordinates
            std::vector<Eigen::MatrixXdRef_ptr> jac_;       //!< The Task Jacobian matrix
            Eigen::VectorXdRef_ptr eff_phi_;                //!< End-effector phi (output of kinematica)
            Eigen::MatrixXdRef_ptr eff_jac_;                //!< End-effector Jacobian (output of kinematica)
            bool getEffReferences();
            int phiCnt_;
            int jacCnt_;
            Eigen::VectorXi phiFlag_;
            Eigen::VectorXi jacFlag_;
	};

	//!< Typedefines for some common functionality
	typedef Factory<std::string, TaskMap> TaskMap_fac;  //!< Task Map Factory
	typedef boost::shared_ptr<TaskMap> TaskMap_ptr;  //!< Task Map smart pointer
	typedef std::map<std::string, TaskMap_ptr> TaskMap_map;  //!< The mapping by name of TaskMaps
}
#endif
