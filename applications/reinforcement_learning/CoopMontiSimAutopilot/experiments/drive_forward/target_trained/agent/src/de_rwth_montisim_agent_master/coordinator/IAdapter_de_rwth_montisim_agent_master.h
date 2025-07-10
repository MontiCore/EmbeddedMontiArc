
#pragma once
#include "de_rwth_montisim_agent_master.h"

class IAdapter_de_rwth_montisim_agent_master{
	public:
		virtual ~IAdapter_de_rwth_montisim_agent_master(){}
		virtual void init(de_rwth_montisim_agent_master* comp) = 0;
		virtual void tick() = 0;
		virtual bool hasReceivedNewData() = 0;
};
