#pragma once
#include "ba_system_collisionDetection.h"

class IAdapter_ba_system_collisionDetection{
	public:
		virtual ~IAdapter_ba_system_collisionDetection(){}
		virtual void init(ba_system_collisionDetection* comp) = 0;
		virtual void tick() = 0;
};
