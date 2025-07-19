/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.TagLatencyTagSchema;
tags TagLatency for oeffentlicher_Demonstrator_FAS_v04.dEMO_FAS.dEMO_FAS.subsystem.dEMO_FAS.dEMO_FAS_Funktion.limiter {
	tag limiter_Function with LatencyCmpInst = 26240 ns;
	tag limiter_Function.limiter_Enabled with LatencyCmpInst = 18400 ns;
	tag limiter_Function.limiter_Enabled.edgeRising with LatencyCmpInst = 4720 ns;
	tag limiter_Function.limiter_Enabled.edgeRising.logOp_A with LatencyCmpInst = 1360 ns; 
	tag limiter_Function.limiter_Enabled.edgeRising.logOp_N with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_Enabled.edgeRising.memory_U with LatencyCmpInst = 0 ns; 
	tag limiter_Function.limiter_Enabled.edgeRising.switch_R with LatencyCmpInst = 2400 ns; 
	tag limiter_Function.limiter_Enabled.ifBlock with LatencyCmpInst = 800 ns; 
	tag limiter_Function.limiter_Enabled.limiter_Active with LatencyCmpInst = 1360 ns;
	tag limiter_Function.limiter_Enabled.limiter_Active.gain with LatencyCmpInst = 160 ns; 
	tag limiter_Function.limiter_Enabled.limiter_Active.logicalOperator with LatencyCmpInst = 1360 ns; 
	tag limiter_Function.limiter_Enabled.limiter_Deactive with LatencyCmpInst = 1360 ns;
	tag limiter_Function.limiter_Enabled.limiter_Deactive.gain with LatencyCmpInst = 160 ns; 
	tag limiter_Function.limiter_Enabled.limiter_Deactive.logicalOperator with LatencyCmpInst = 1360 ns; 
	tag limiter_Function.limiter_Enabled.logicalOperator1 with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_Enabled.logicalOperator2 with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_Enabled.rSFlipFlop with LatencyCmpInst = 10560 ns;
	tag limiter_Function.limiter_Enabled.rSFlipFlop.logOp_N with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_Enabled.rSFlipFlop.memory_Q with LatencyCmpInst = 0 ns; 
	tag limiter_Function.limiter_Enabled.rSFlipFlop.switch_R with LatencyCmpInst = 2400 ns; 
	tag limiter_Function.limiter_Enabled.rSFlipFlop.switch_S with LatencyCmpInst = 2400 ns; 
	tag limiter_Function.limiter_Enabled.terminator with LatencyCmpInst = 0 ns; 
	tag limiter_Function.limiter_InitialSetValue with LatencyCmpInst = 0 ns;
	tag limiter_Function.limiter_SetValue with LatencyCmpInst = 3200 ns;
	tag limiter_Function.limiter_SetValue.relOp1 with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_SetValue.relOp3 with LatencyCmpInst = 960 ns; 
	tag limiter_Function.limiter_SetValue.v_LimSetValueMinus with LatencyCmpInst = 640 ns;
	tag limiter_Function.limiter_SetValue.v_LimSetValueMinus.sum with LatencyCmpInst = 640 ns; 
	tag limiter_Function.limiter_SetValue.v_LimSetValuePlus with LatencyCmpInst = 640 ns;
	tag limiter_Function.limiter_SetValue.v_LimSetValuePlus.sum with LatencyCmpInst = 640 ns; 
	tag limiter_Function.limiter_StartUpSetValue with LatencyCmpInst = 0 ns;
	tag limiter_Function.logicalOperator with LatencyCmpInst = 1360 ns; 
	tag limiter_Function.minMax with LatencyCmpInst = 2240 ns; 
	tag limiter_Function.switchBlock with LatencyCmpInst = 2400 ns; 
}
