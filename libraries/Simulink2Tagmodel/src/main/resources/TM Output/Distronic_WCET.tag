/* (c) https://github.com/MontiCore/monticore */
package model;
conforms to nfp.TagLatencyTagSchema;
tags TagLatency for oeffentlicher_Demonstrator_FAS_v04.dEMO_FAS.dEMO_FAS.subsystem.dEMO_FAS.dEMO_FAS_Funktion.distronic {
	tag distronic_Deactive with LatencyCmpInst = 0 ns;
	tag distronic_Disabled with LatencyCmpInst = 0 ns;
	tag distronic_Enabled with LatencyCmpInst = 16640 ns;
	tag distronic_Enabled.lookUpTable with LatencyCmpInst = 5000 ns; 
	tag distronic_Enabled.lookUpTable1 with LatencyCmpInst = 5000 ns; 
	tag distronic_Enabled.lookUpTable2 with LatencyCmpInst = 5000 ns; 
	tag distronic_Enabled.lookUpTable3 with LatencyCmpInst = 5000 ns; 
	tag distronic_Enabled.lookUpTable4 with LatencyCmpInst = 5000 ns; 
	tag distronic_Enabled.mul with LatencyCmpInst = 1120 ns; 
	tag distronic_Enabled.mul1 with LatencyCmpInst = 1120 ns; 
	tag distronic_Enabled.mul2 with LatencyCmpInst = 1120 ns; 
	tag distronic_Enabled.multiportSwitch with LatencyCmpInst = 5760 ns; 
	tag distronic_Enabled.relOp with LatencyCmpInst = 960 ns; 
	tag distronic_Enabled.sum with LatencyCmpInst = 640 ns; 
	tag distronic_Enabled.switchBlock with LatencyCmpInst = 3000 ns; 
	tag distronic_FTS_Enabled with LatencyCmpInst = 9760 ns;
	tag distronic_FTS_Enabled.lookUpTable with LatencyCmpInst = 5000 ns; 
	tag distronic_FTS_Enabled.lookUpTable1 with LatencyCmpInst = 5000 ns; 
	tag distronic_FTS_Enabled.mul1 with LatencyCmpInst = 1120 ns; 
	tag distronic_FTS_Enabled.relOp with LatencyCmpInst = 960 ns; 
	tag distronic_FTS_Enabled.sum with LatencyCmpInst = 640 ns; 
	tag distronic_FTS_Enabled.switchBlock with LatencyCmpInst = 3000 ns; 
	tag ifBlock with LatencyCmpInst = 4880 ns; 
}
