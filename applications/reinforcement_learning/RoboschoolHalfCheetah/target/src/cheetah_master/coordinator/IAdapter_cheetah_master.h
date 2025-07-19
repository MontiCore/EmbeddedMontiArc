#pragma once
#include "cheetah_master.h"

class IAdapter_cheetah_master{
	public:
		virtual ~IAdapter_cheetah_master(){}
		virtual void init(cheetah_master* comp) = 0;
		virtual void tick() = 0;
};
