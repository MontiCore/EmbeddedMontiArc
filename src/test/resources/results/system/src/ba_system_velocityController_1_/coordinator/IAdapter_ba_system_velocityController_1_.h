#pragma once
#include "ba_system_velocityController_1_.h"

class IAdapter_ba_system_velocityController_1_{
	public:
		virtual ~IAdapter_ba_system_velocityController_1_(){}
		virtual void init(ba_system_velocityController_1_* comp) = 0;
		virtual void tick() = 0;
};
