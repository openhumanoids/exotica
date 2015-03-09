/*
 * AICOexample.h
 *
 *  Created on: 30 Apr 2014
 *      Author: s0972326
 */

#ifndef AICOEXAMPLE_H_
#define AICOEXAMPLE_H_

#include "aico/AICOsolver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <kinematic_maps/EffPosition.h>
#include <exotica/Initialiser.h>

namespace exotica
{

	class AICOexample
	{
		public:
			AICOexample ();
			virtual
			~AICOexample ();
	};

} /* namespace exotica */

#endif /* AICOEXAMPLE_H_ */
