#pragma once
#include "ba_system_velocityController_2_.h"

class IAdapter_ba_system_velocityController_2_{
	public:
		virtual ~IAdapter_ba_system_velocityController_2_(){}
		virtual void init(ba_system_velocityController_2_* comp) = 0;
		virtual void tick() = 0;
};
