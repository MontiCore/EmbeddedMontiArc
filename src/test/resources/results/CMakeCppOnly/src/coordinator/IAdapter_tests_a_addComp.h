#pragma once
#include "tests_a_addComp.h"

class IAdapter_tests_a_addComp{
	public:
		virtual ~IAdapter_tests_a_addComp(){}
		virtual void init(tests_a_addComp* comp) = 0;
		virtual void tick() = 0;
};
