#pragma once
#include "ba_system_intersectionController.h"

class IAdapter_ba_system_intersectionController{
	public:
		virtual ~IAdapter_ba_system_intersectionController(){}
		virtual void init(ba_system_intersectionController* comp) = 0;
		virtual void tick() = 0;
};
