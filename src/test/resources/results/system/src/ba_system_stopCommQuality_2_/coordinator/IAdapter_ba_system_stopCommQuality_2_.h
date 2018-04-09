#pragma once
#include "ba_system_stopCommQuality_2_.h"

class IAdapter_ba_system_stopCommQuality_2_{
	public:
		virtual ~IAdapter_ba_system_stopCommQuality_2_(){}
		virtual void init(ba_system_stopCommQuality_2_* comp) = 0;
		virtual void tick() = 0;
};
