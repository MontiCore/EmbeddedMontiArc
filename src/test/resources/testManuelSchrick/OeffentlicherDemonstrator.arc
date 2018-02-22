package testManuelSchrick;

<<Type = "SubSystem">> component OeffentlicherDemonstrator {

	<<Type = "SubSystem">> component DEMO_FAS {
		port
			in Double dEMOFAS_INIn1,
			out Double dEMOFAS_OUTOut1;

		<<Type = "BusCreator">> component BusCreator {
			port
				<<BusSignal = "CC_active_b">> in Double in1,
				<<BusSignal = "Acceleration_pc">> in Double in2,
				<<BusSignal = "BrakeForce_pc">> in Double in3,
				<<BusSignal = "CCSetValue_kmh">> in Double in4,
				<<BusSignal = "LimiterSetValue_kmh">> in Double in5,
				<<BusSignal = "Limiter_active_b">> in Double in6,
				<<BusSignal = "Opti_warn_b">> in Double in7,
				<<BusSignal = "Acusti_warn_b">> in Double in8,
				out Double out1;
		}
		component BusCreator busCreator;

		<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
			port
				in Double in1,
				<<BusSignal = "ParkingBrake_b">> out Double parkingBrake_bOut1,
				<<BusSignal = "BrakeForce_pedal_pc">> out Double brakeForce_pedal_pcOut2,
				<<BusSignal = "Accelaration_pedal_pc">> out Double accelaration_pedal_pcOut3,
				<<BusSignal = "CruiseControl_b">> out Double cruiseControl_bOut4,
				<<BusSignal = "Limiter_b">> out Double limiter_bOut5,
				<<BusSignal = "LeverUp_stat">> out Double leverUp_statOut6,
				<<BusSignal = "LeverDown_stat">> out Double leverDown_statOut7,
				<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut8,
				<<BusSignal = "V_Sign_kmh">> out Double v_Sign_kmhOut9,
				<<BusSignal = "Sign_b">> out Double sign_bOut10,
				<<BusSignal = "Distance_stat">> out Double distance_statOut11,
				<<BusSignal = "V_Obj_rel_kmh">> out Double v_Obj_rel_kmhOut12,
				<<BusSignal = "Distance_Object_m">> out Double distance_Object_mOut13;
		}
		component BusSelector busSelector;

		<<Type = "SubSystem">> component DEMO_FAS {
			port
				in Double _ParkingBrake_bIn1,
				in Double _BrakeForce_pedal_pcIn2,
				in Double _Accelaration_pedal_pcIn3,
				in Double _CruiseControl_bIn4,
				in Double _Limiter_bIn5,
				in Double _LeverUp_statIn6,
				in Double _LeverDown_statIn7,
				in Double _V_Vehicle_kmhIn8,
				in Double _V_Sign_kmhIn9,
				in Double _Sign_bIn10,
				in Double _Distance_statIn11,
				in Double _V_Obj_rel_kmhIn12,
				in Double _Distance_Object_mIn13,
				out Double _CC_active_bOut1,
				out Double _Acceleration_pcOut2,
				out Double _BrakeForce_pcOut3,
				out Double _CCSetValue_kmhOut4,
				out Double _LimiterSetValue_kmhOut5,
				out Double _Limiter_active_bOut6,
				out Double _Opti_warn_bOut7,
				out Double _Acusti_warn_bOut8;

			<<Type = "SubSystem">> component Subsystem {
				port
					in Double _ParkingBrake_bIn1,
					in Double _BrakeForce_pedal_pcIn2,
					in Double _Accelaration_pedal_pcIn3,
					in Double _CruiseControl_bIn4,
					in Double _Limiter_bIn5,
					in Double _LeverUp_statIn6,
					in Double _LeverDown_statIn7,
					in Double _V_Vehicle_kmhIn8,
					in Double _V_Sign_kmhIn9,
					in Double _Sign_bIn10,
					in Double _Distance_statIn11,
					in Double _V_Obj_rel_kmhIn12,
					in Double _Distance_Object_mIn13,
					out Double _CC_active_bOut1,
					out Double _Acceleration_pcOut2,
					out Double _BrakeForce_pcOut3,
					out Double _CCSetValue_kmhOut4,
					out Double _LimiterSetValue_kmhOut5,
					out Double _Limiter_active_bOut6,
					out Double _Opti_warn_bOut7,
					out Double _Acusti_warn_bOut8;

				<<Type = "SubSystem">> component DEMO_FAS {
					port
						in Double _ParkingBrake_bIn1,
						in Double _BrakeForce_pedal_pcIn2,
						in Double _Accelaration_pedal_pcIn3,
						in Double _CruiseControl_bIn4,
						in Double _Limiter_bIn5,
						in Double _LeverUp_statIn6,
						in Double _LeverDown_statIn7,
						in Double _V_Vehicle_kmhIn8,
						in Double _V_Sign_kmhIn9,
						in Double _Sign_bIn10,
						in Double _Distance_statIn11,
						in Double _V_Obj_rel_kmhIn12,
						in Double _Distance_Object_mIn13,
						out Double _CC_active_bOut1,
						out Double _Acceleration_pcOut2,
						out Double _BrakeForce_pcOut3,
						out Double _CCSetValue_kmhOut4,
						out Double _LimiterSetValue_kmhOut5,
						out Double _Limiter_active_bOut6,
						out Double _Opti_warn_bOut7,
						out Double _Acusti_warn_bOut8;

					<<Type = "SubSystem">> component DEMO_FAS_Funktion {
						port
							in Double inputBusIn1,
							out Double outputBusOut1;

						<<Type = "SubSystem">> component BrakeAssistant {
							port
								in Double brakeForce_pedal_pcIn1,
								out Double brakeForceBoosted_pcOut1;

							<<Type = "SubSystem">> component Brake_Booster {
								port
									in Double brakeForce_pedal_pcIn1,
									out Double brakeForceBoosted_pcOut1;

								<<Type = "Constant",Value = "100">> component Constant {
									port
										out Double out1;
								}
								component Constant constant;

								<<Type = "Constant",Value = "70">> component DEMO_FAS_BrakeBooster_Threshold {
									port
										out Double out1;
								}
								component DEMO_FAS_BrakeBooster_Threshold dEMO_FAS_BrakeBooster_Threshold;

								<<Type = "RelationalOperator",Operator = ">=">> component RelOp {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component RelOp relOp;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double brakeForceBoosted_pcOut1;
								}
								component SwitchBlock switchBlock;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect switchBlock.brakeForceBoosted_pcOut1 -> brakeForceBoosted_pcOut1;
								connect constant.out1 -> switchBlock.ifIn;
								connect dEMO_FAS_BrakeBooster_Threshold.out1 -> relOp.in2;
								connect relOp.out1 -> switchBlock.condition;
								connect brakeForce_pedal_pcIn1 -> relOp.in1;
								connect brakeForce_pedal_pcIn1 -> switchBlock.elseIn;
							}
							component Brake_Booster brake_Booster;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect brake_Booster.brakeForceBoosted_pcOut1 -> brakeForceBoosted_pcOut1;
							connect brakeForce_pedal_pcIn1 -> brake_Booster.brakeForce_pedal_pcIn1;
						}
						component BrakeAssistant brakeAssistant;

						<<Type = "BusCreator">> component BusCreator {
							port
								<<BusSignal = "CC_active_b">> in Double cC_active_bIn1,
								<<BusSignal = "Acceleration_pc">> in Double in2,
								<<BusSignal = "BrakeForce_pc">> in Double in3,
								<<BusSignal = "CCSetValue_kmh">> in Double cCSetValue_kmhIn4,
								<<BusSignal = "LimiterSetValue_kmh">> in Double limiterSetValue_kmhIn5,
								<<BusSignal = "Limiter_active_b">> in Double limiter_active_bIn6,
								<<BusSignal = "Opti_warn_b">> in Double opti_warn_bIn7,
								<<BusSignal = "Acusti_warn_b">> in Double acusti_warn_bIn8,
								out Double outputBusOut1;
						}
						component BusCreator busCreator;

						<<Type = "BusCreator">> component BusCreator1 {
							port
								<<BusSignal = "V_CC_delta_kmh">> in Double in1,
								<<BusSignal = "CC_active_b">> in Double in2,
								out Double out1;
						}
						component BusCreator1 busCreator1;

						<<Type = "BusCreator">> component BusCreator2 {
							port
								<<BusSignal = "VMax_kmh">> in Double in1,
								<<BusSignal = "Limiter_active_b">> in Double in2,
								out Double out1;
						}
						component BusCreator2 busCreator2;

						<<Type = "BusCreator">> component BusCreator3 {
							port
								<<BusSignal = "Opti_warn_b">> in Double in1,
								<<BusSignal = "Acusti_warn_b">> in Double in2,
								out Double out1;
						}
						component BusCreator3 busCreator3;

						<<Type = "BusCreator">> component BusCreator4 {
							port
								<<BusSignal = "BrakeForce_Emergency_pc">> in Double in1,
								<<BusSignal = "Acusti_warn_b">> in Double in2,
								out Double out1;
						}
						component BusCreator4 busCreator4;

						<<Type = "BusCreator">> component BusCreator5 {
							port
								<<BusSignal = "FTS_active_b">> in Double in1,
								<<BusSignal = "FTS_Abstand_soll_m">> in Double in2,
								out Double out1;
						}
						component BusCreator5 busCreator5;

						<<Type = "BusCreator">> component BusCreator6 {
							port
								<<BusSignal = "BrakeForce_Distronic_pc">> in Double in1,
								<<BusSignal = "Decelerator_pc">> in Double in2,
								out Double out1;
						}
						component BusCreator6 busCreator6;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
							port
								in Double in1,
								<<BusSignal = "ParkingBrake_b">> out Double parkingBrake_bOut1,
								<<BusSignal = "BrakeForce_pedal_pc">> out Double brakeForce_pedal_pcOut2,
								<<BusSignal = "CruiseControl_b">> out Double cruiseControl_bOut3,
								<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut4,
								<<BusSignal = "Limiter_b">> out Double limiter_bOut5,
								<<BusSignal = "LeverUp_stat">> out Double leverUp_statOut6,
								<<BusSignal = "LeverDown_stat">> out Double leverDown_statOut7;
						}
						component BusSelector busSelector;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector1 {
							port
								in Double in1,
								<<BusSignal = "Limiter_b">> out Double limiter_bOut1,
								<<BusSignal = "Accelaration_pedal_pc">> out Double accelaration_pedal_pcOut2,
								<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut3,
								<<BusSignal = "LeverUp_stat">> out Double leverUp_statOut4,
								<<BusSignal = "LeverDown_stat">> out Double leverDown_statOut5,
								<<BusSignal = "V_Sign_kmh">> out Double v_Sign_kmhOut6,
								<<BusSignal = "Sign_b">> out Double sign_bOut7;
						}
						component BusSelector1 busSelector1;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector10 {
							port
								in Double in1,
								<<BusSignal = "V_Vehicle_ms">> out Double v_Vehicle_msOut1,
								<<BusSignal = "Distance_Object_m">> out Double distance_Object_mOut2,
								<<BusSignal = "Distance_stat">> out Double distance_statOut3,
								<<BusSignal = "V_Obj_rel_kmh">> out Double v_Obj_rel_kmhOut4;
						}
						component BusSelector10 busSelector10;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector11 {
							port
								in Double in1,
								<<BusSignal = "V_Vehicle_ms">> out Double v_Vehicle_msOut1,
								<<BusSignal = "Distance_Object_m">> out Double distance_Object_mOut2,
								<<BusSignal = "V_Obj_rel_ms">> out Double v_Obj_rel_msOut3;
						}
						component BusSelector11 busSelector11;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector12 {
							port
								in Double in1,
								<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut1,
								<<BusSignal = "V_Obj_rel_kmh">> out Double v_Obj_rel_kmhOut2;
						}
						component BusSelector12 busSelector12;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector13 {
							port
								in Double in1,
								<<BusSignal = "Acusti_warn_b">> out Double acusti_warn_bOut1;
						}
						component BusSelector13 busSelector13;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector14 {
							port
								in Double in1,
								<<BusSignal = "BrakeForce_Emergency_pc">> out Double brakeForce_Emergency_pcOut1;
						}
						component BusSelector14 busSelector14;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector15 {
							port
								in Double in1,
								<<BusSignal = "FTS_active_b">> out Double fTS_active_bOut1,
								<<BusSignal = "FTS_Abstand_soll_m">> out Double fTS_Abstand_soll_mOut2;
						}
						component BusSelector15 busSelector15;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector16 {
							port
								in Double in1,
								<<BusSignal = "FTS_active_b">> out Double fTS_active_bOut1;
						}
						component BusSelector16 busSelector16;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector17 {
							port
								in Double in1,
								<<BusSignal = "BrakeForce_Distronic_pc">> out Double brakeForce_Distronic_pcOut1,
								<<BusSignal = "Decelerator_pc">> out Double decelerator_pcOut2;
						}
						component BusSelector17 busSelector17;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector18 {
							port
								in Double in1,
								<<BusSignal = "CC_active_b">> out Double cC_active_bOut1;
						}
						component BusSelector18 busSelector18;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector2 {
							port
								in Double in1,
								<<BusSignal = "Accelaration_pedal_pc">> out Double accelaration_pedal_pcOut1,
								<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut2,
								<<BusSignal = "BrakeForce_pedal_pc">> out Double brakeForce_pedal_pcOut3;
						}
						component BusSelector2 busSelector2;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector3 {
							port
								in Double in1,
								<<BusSignal = "CC_active_b">> out Double cC_active_bOut1;
						}
						component BusSelector3 busSelector3;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector4 {
							port
								in Double in1,
								<<BusSignal = "V_CC_delta_kmh">> out Double v_CC_delta_kmhOut1;
						}
						component BusSelector4 busSelector4;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector5 {
							port
								in Double in1,
								<<BusSignal = "VMax_kmh">> out Double vMax_kmhOut1;
						}
						component BusSelector5 busSelector5;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector6 {
							port
								in Double in1,
								<<BusSignal = "Limiter_active_b">> out Double limiter_active_bOut1;
						}
						component BusSelector6 busSelector6;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector7 {
							port
								in Double in1,
								<<BusSignal = "BrakeForce_pedal_pc">> out Double brakeForce_pedal_pcOut1;
						}
						component BusSelector7 busSelector7;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector8 {
							port
								in Double in1,
								<<BusSignal = "V_Vehicle_ms">> out Double v_Vehicle_msOut1,
								<<BusSignal = "Distance_Object_m">> out Double distance_Object_mOut2;
						}
						component BusSelector8 busSelector8;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector9 {
							port
								in Double in1,
								<<BusSignal = "Opti_warn_b">> out Double opti_warn_bOut1,
								<<BusSignal = "Acusti_warn_b">> out Double acusti_warn_bOut2;
						}
						component BusSelector9 busSelector9;

						<<Type = "DataStoreMemory",Variable = "DEMO_FAS_V_CCSetValue",InitialValue = "-1">> component DEMO_FAS_V_CCSetValue {
						}
						component DEMO_FAS_V_CCSetValue dEMO_FAS_V_CCSetValue;

						<<Type = "DataStoreMemory",Variable = "DEMO_FAS_V_LimiterSetValue",InitialValue = "0">> component DEMO_FAS_V_LimiterSetValue {
						}
						component DEMO_FAS_V_LimiterSetValue dEMO_FAS_V_LimiterSetValue;

						<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreRead1 {
							port
								out Double cCSetValue_kmhOut1;
						}
						component DataStoreRead1 dataStoreRead1;

						<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreRead2 {
							port
								out Double limiterSetValue_kmhOut1;
						}
						component DataStoreRead2 dataStoreRead2;

						<<Type = "SubSystem">> component Distancewarner {
							port
								in Double v_Vehicle_msIn1,
								in Double distance_Object_mIn2,
								out Double opti_warn_bOut1,
								out Double acusti_warn_bOut2;

							<<Type = "SubSystem">> component Distancewarner_Function {
								port
									in Double v_Vehicle_msIn1,
									in Double distance_Object_mIn2,
									out Double opti_warn_bOut1,
									out Double acusti_warn_bOut2;

								<<Type = "Constant",Value = "1">> component DEMO_FAS_Accusti_Threshold_Time {
									port
										out Double out1;
								}
								component DEMO_FAS_Accusti_Threshold_Time dEMO_FAS_Accusti_Threshold_Time;

								<<Type = "Constant",Value = "1.5">> component DEMO_FAS_Opti_Threshold_Time {
									port
										out Double out1;
								}
								component DEMO_FAS_Opti_Threshold_Time dEMO_FAS_Opti_Threshold_Time;

								<<Type = "Product",Inputs = "**">> component Mul {
									port
										in Double in1,
										in Double in2,
										out Double opti_warn_bOut1;
								}
								component Mul mul;

								<<Type = "Product",Inputs = "**">> component Mul1 {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Mul1 mul1;

								<<Type = "RelationalOperator",Operator = "<=">> component RelOp {
									port
										in Double in1,
										in Double opti_warn_bIn2,
										out Double opti_warn_bOut1;
								}
								component RelOp relOp;

								<<Type = "RelationalOperator",Operator = "<=">> component RelOp2 {
									port
										in Double in1,
										in Double in2,
										out Double acusti_warn_bOut1;
								}
								component RelOp2 relOp2;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect mul1.out1 -> relOp2.in2;
								connect dEMO_FAS_Accusti_Threshold_Time.out1 -> mul1.in2;
								connect relOp2.acusti_warn_bOut1 -> acusti_warn_bOut2;
								connect distance_Object_mIn2 -> relOp.in1;
								connect distance_Object_mIn2 -> relOp2.in1;
								connect mul.opti_warn_bOut1 -> relOp.opti_warn_bIn2;
								connect relOp.opti_warn_bOut1 -> opti_warn_bOut1;
								connect dEMO_FAS_Opti_Threshold_Time.out1 -> mul.in2;
								connect v_Vehicle_msIn1 -> mul1.in1;
								connect v_Vehicle_msIn1 -> mul.in1;
							}
							component Distancewarner_Function distancewarner_Function;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect distance_Object_mIn2 -> distancewarner_Function.distance_Object_mIn2;
							connect v_Vehicle_msIn1 -> distancewarner_Function.v_Vehicle_msIn1;
							connect distancewarner_Function.acusti_warn_bOut2 -> acusti_warn_bOut2;
							connect distancewarner_Function.opti_warn_bOut1 -> opti_warn_bOut1;
						}
						component Distancewarner distancewarner;

						<<Type = "SubSystem">> component Distronic {
							port
								in Double cC_active_bIn1,
								in Double v_Vehicle_msIn2,
								in Double distance_Object_mIn3,
								in Double distance_statIn4,
								in Double v_Obj_rel_kmhIn5,
								in Double fTS_active_bIn6,
								in Double fTS_Abstand_soll_mIn7,
								out Double brakeForce_Distronic_pcOut1,
								out Double decelerator_pcOut2;

							<<Type = "BusCreator">> component BusCreator1 {
								port
									<<BusSignal = "BrakeForce_Distronic_pc">> in Double in1,
									<<BusSignal = "Decelerator_pc">> in Double in2,
									out Double out1;
							}
							component BusCreator1 busCreator1;

							<<Type = "BusCreator">> component BusCreator2 {
								port
									<<BusSignal = "BrakeForce_Distronic_pc">> in Double in1,
									<<BusSignal = "Decelerator_pc">> in Double in2,
									out Double out1;
							}
							component BusCreator2 busCreator2;

							<<Type = "BusCreator">> component BusCreator3 {
								port
									<<BusSignal = "BrakeForce_Distronic_pc">> in Double in1,
									<<BusSignal = "Decelerator_pc">> in Double in2,
									out Double out1;
							}
							component BusCreator3 busCreator3;

							<<Type = "BusCreator">> component BusCreator4 {
								port
									<<BusSignal = "BrakeForce_Distronic_pc">> in Double in1,
									<<BusSignal = "Decelerator_pc">> in Double in2,
									out Double out1;
							}
							component BusCreator4 busCreator4;

							<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
								port
									in Double brakeForce_Distronic_pcIn1,
									<<BusSignal = "BrakeForce_Distronic_pc">> out Double brakeForce_Distronic_pcOut1,
									<<BusSignal = "Decelerator_pc">> out Double decelerator_pcOut2;
							}
							component BusSelector busSelector;

							<<Type = "ActionSubSystem">> component Distronic_Deactive {
								port
									in Double ifactionIn,
									<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double brakeForce_Distronic_pcOut1,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double decelerator_pcOut2;

								<<Type = "Constant",Value = "0">> component Constant {
									port
										out Double brakeForce_Distronic_pcOut1;
								}
								component Constant constant;

								<<Type = "Constant",Value = "100">> component Constant1 {
									port
										out Double decelerator_pcOut1;
								}
								component Constant1 constant1;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect constant1.decelerator_pcOut1 -> decelerator_pcOut2;
								connect constant.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
							}
							component Distronic_Deactive distronic_Deactive;

							<<Type = "ActionSubSystem">> component Distronic_Disabled {
								port
									in Double ifactionIn,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double brakeForce_Distronic_pcOut1,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double decelerator_pcOut2;

								<<Type = "Constant",Value = "0">> component Constant {
									port
										out Double brakeForce_Distronic_pcOut1;
								}
								component Constant constant;

								<<Type = "Constant",Value = "100">> component Constant1 {
									port
										out Double decelerator_pcOut1;
								}
								component Constant1 constant1;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect constant1.decelerator_pcOut1 -> decelerator_pcOut2;
								connect constant.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
							}
							component Distronic_Disabled distronic_Disabled;

							<<Type = "ActionSubSystem">> component Distronic_Enabled {
								port
									in Double v_Vehicle_msIn1,
									in Double distance_Object_mIn2,
									in Double distance_statIn3,
									in Double v_Obj_rel_kmhIn4,
									in Double ifactionIn,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double brakeForce_Distronic_pcOut1,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double decelerator_pcOut2;

								<<Type = "Constant",Value = "100">> component Constant {
									port
										out Double out1;
								}
								component Constant constant;

								<<Type = "Constant",Value = "2.5">> component Constant1 {
									port
										out Double out1;
								}
								component Constant1 constant1;

								<<Type = "Constant",Value = "3">> component DEMO_FAS_Distance_Treshold_Far {
									port
										out Double out1;
								}
								component DEMO_FAS_Distance_Treshold_Far dEMO_FAS_Distance_Treshold_Far;

								<<Type = "Constant",Value = "2.5">> component DEMO_FAS_Distance_Treshold_Med {
									port
										out Double out1;
								}
								component DEMO_FAS_Distance_Treshold_Med dEMO_FAS_Distance_Treshold_Med;

								<<Type = "Constant",Value = "2">> component DEMO_FAS_Distance_Treshold_Near {
									port
										out Double out1;
								}
								component DEMO_FAS_Distance_Treshold_Near dEMO_FAS_Distance_Treshold_Near;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[100,100, 10,0]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[0,5, 20,50]">> component LookUpTable {
									port
										in Double in1,
										out Double out1;
								}
								component LookUpTable lookUpTable;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[0,0.15,1]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[-1,0,10]">> component LookUpTable1 {
									port
										in Double in1,
										out Double out1;
								}
								component LookUpTable1 lookUpTable1;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[0,50,90,100]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[0,20,50,100]">> component LookUpTable2 {
									port
										in Double in1,
										out Double decelerator_pcOut1;
								}
								component LookUpTable2 lookUpTable2;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[1,0.9]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[-10,0]">> component LookUpTable3 {
									port
										in Double in1,
										out Double out1;
								}
								component LookUpTable3 lookUpTable3;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[50,0]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[0,10]">> component LookUpTable4 {
									port
										in Double in1,
										out Double out1;
								}
								component LookUpTable4 lookUpTable4;

								<<Type = "Product",Inputs = "**">> component Mul {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Mul mul;

								<<Type = "Product",Inputs = "**">> component Mul1 {
									port
										in Double in1,
										in Double in2,
										out Double brakeForce_Distronic_pcOut1;
								}
								component Mul1 mul1;

								<<Type = "Product",Inputs = "**">> component Mul2 {
									port
										in Double decelerator_pcIn1,
										in Double in2,
										out Double decelerator_pcOut1;
								}
								component Mul2 mul2;

								<<Type = "MultiPortSwitch",DataPortOrder = "One-based contiguous">> component MultiportSwitch {
									port
										in Double in1,
										in Double in2,
										in Double in3,
										in Double in4,
										out Double out1;
								}
								component MultiportSwitch multiportSwitch;

								<<Type = "RelationalOperator",Operator = "<">> component RelOp {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component RelOp relOp;

								<<Type = "Sum",ListOfSigns = "+-+">> component Sum {
									port
										in Double in1,
										in Double in2,
										in Double in3,
										out Double out1;
								}
								component Sum sum;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double brakeForce_Distronic_pcOut1;
								}
								component SwitchBlock switchBlock;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect lookUpTable4.out1 -> sum.in3;
								connect lookUpTable3.out1 -> mul2.in2;
								connect lookUpTable2.decelerator_pcOut1 -> mul2.decelerator_pcIn1;
								connect mul2.decelerator_pcOut1 -> decelerator_pcOut2;
								connect constant1.out1 -> relOp.in2;
								connect relOp.out1 -> switchBlock.condition;
								connect constant.out1 -> switchBlock.ifIn;
								connect distance_Object_mIn2 -> sum.in1;
								connect distance_Object_mIn2 -> relOp.in1;
								connect switchBlock.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
								connect lookUpTable.out1 -> mul1.in1;
								connect distance_statIn3 -> multiportSwitch.in1;
								connect dEMO_FAS_Distance_Treshold_Far.out1 -> multiportSwitch.in2;
								connect dEMO_FAS_Distance_Treshold_Near.out1 -> multiportSwitch.in4;
								connect dEMO_FAS_Distance_Treshold_Med.out1 -> multiportSwitch.in3;
								connect multiportSwitch.out1 -> mul.in2;
								connect v_Vehicle_msIn1 -> mul.in1;
								connect v_Obj_rel_kmhIn4 -> lookUpTable3.in1;
								connect v_Obj_rel_kmhIn4 -> lookUpTable1.in1;
								connect v_Obj_rel_kmhIn4 -> lookUpTable4.in1;
								connect mul.out1 -> sum.in2;
								connect sum.out1 -> lookUpTable2.in1;
								connect sum.out1 -> lookUpTable.in1;
								connect lookUpTable1.out1 -> mul1.in2;
								connect mul1.brakeForce_Distronic_pcOut1 -> switchBlock.elseIn;
							}
							component Distronic_Enabled distronic_Enabled;

							<<Type = "ActionSubSystem">> component Distronic_FTS_Enabled {
								port
									in Double distance_Object_mIn1,
									in Double fTS_Abstand_soll_mIn2,
									in Double v_Obj_rel_kmhIn3,
									in Double ifactionIn,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double brakeForce_Distronic_pcOut1,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double decelerator_pcOut2;

								<<Type = "Constant",Value = "100">> component Constant {
									port
										out Double out1;
								}
								component Constant constant;

								<<Type = "Constant",Value = "2.5">> component Constant1 {
									port
										out Double out1;
								}
								component Constant1 constant1;

								<<Type = "Constant",Value = "100">> component Constant2 {
									port
										out Double decelerator_pcOut1;
								}
								component Constant2 constant2;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[100,100, 10,0]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[0,2, 10,20]">> component LookUpTable {
									port
										in Double in1,
										out Double brakeForce_Distronic_pcOut1;
								}
								component LookUpTable lookUpTable;

								<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[0,0.15,1]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[-1,0,10]">> component LookUpTable1 {
									port
										in Double in1,
										out Double out1;
								}
								component LookUpTable1 lookUpTable1;

								<<Type = "Product",Inputs = "**">> component Mul1 {
									port
										in Double brakeForce_Distronic_pcIn1,
										in Double in2,
										out Double brakeForce_Distronic_pcOut1;
								}
								component Mul1 mul1;

								<<Type = "RelationalOperator",Operator = "<">> component RelOp {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component RelOp relOp;

								<<Type = "Sum",ListOfSigns = "+-">> component Sum {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Sum sum;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double brakeForce_Distronic_pcOut1;
								}
								component SwitchBlock switchBlock;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect constant2.decelerator_pcOut1 -> decelerator_pcOut2;
								connect constant1.out1 -> relOp.in2;
								connect relOp.out1 -> switchBlock.condition;
								connect mul1.brakeForce_Distronic_pcOut1 -> switchBlock.elseIn;
								connect constant.out1 -> switchBlock.ifIn;
								connect lookUpTable.brakeForce_Distronic_pcOut1 -> mul1.brakeForce_Distronic_pcIn1;
								connect lookUpTable1.out1 -> mul1.in2;
								connect v_Obj_rel_kmhIn3 -> lookUpTable1.in1;
								connect fTS_Abstand_soll_mIn2 -> sum.in2;
								connect sum.out1 -> lookUpTable.in1;
								connect distance_Object_mIn1 -> sum.in1;
								connect distance_Object_mIn1 -> relOp.in1;
								connect switchBlock.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
							}
							component Distronic_FTS_Enabled distronic_FTS_Enabled;

							<<Type = "If",Expression = "u1 & u2 & ~u3,u1 & u2 & u3 ,u1 & ~u2",ShowElse = "on">> component IfBlock {
								port
									in Double condition1,
									in Double condition2,
									in Double condition3,
									out Double action1,
									out Double action2,
									out Double action3,
									out Double action4;
							}
							component IfBlock ifBlock;

							<<Type = "Merge",InitialOutput = "[]">> component Merge {
								port
									in Double in1,
									in Double in2,
									in Double in3,
									in Double in4,
									out Double brakeForce_Distronic_pcOut1;
							}
							component Merge merge;

							<<Type = "SubSystem">> component VAR_TL_Constant {
								port
									out Double out1Out1;

								<<Type = "Constant",Value = "VAR_DEMO_FAS_Distronic">> component VAR_TL_Constant {
									port
										out Double out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								connect vAR_TL_Constant.out1 -> out1Out1;
							}
							component VAR_TL_Constant vAR_TL_Constant;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect vAR_TL_Constant.out1Out1 -> ifBlock.condition1;
							connect busSelector.decelerator_pcOut2 -> decelerator_pcOut2;
							connect distronic_FTS_Enabled.decelerator_pcOut2 -> busCreator4.in2;
							connect distronic_FTS_Enabled.brakeForce_Distronic_pcOut1 -> busCreator4.in1;
							connect busCreator4.out1 -> merge.in2;
							connect merge.brakeForce_Distronic_pcOut1 -> busSelector.brakeForce_Distronic_pcIn1;
							connect distronic_Deactive.decelerator_pcOut2 -> busCreator3.in2;
							connect distronic_Deactive.brakeForce_Distronic_pcOut1 -> busCreator3.in1;
							connect busCreator3.out1 -> merge.in4;
							connect busCreator2.out1 -> merge.in3;
							connect distronic_Disabled.decelerator_pcOut2 -> busCreator2.in2;
							connect distronic_Disabled.brakeForce_Distronic_pcOut1 -> busCreator2.in1;
							connect distronic_Enabled.decelerator_pcOut2 -> busCreator1.in2;
							connect busCreator1.out1 -> merge.in1;
							connect distronic_Enabled.brakeForce_Distronic_pcOut1 -> busCreator1.in1;
							connect fTS_Abstand_soll_mIn7 -> distronic_FTS_Enabled.fTS_Abstand_soll_mIn2;
							connect ifBlock.action3 -> distronic_Disabled.ifactionIn;
							connect fTS_active_bIn6 -> ifBlock.condition3;
							connect busSelector.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
							connect ifBlock.action4 -> distronic_Deactive.ifactionIn;
							connect ifBlock.action2 -> distronic_FTS_Enabled.ifactionIn;
							connect ifBlock.action1 -> distronic_Enabled.ifactionIn;
							connect distance_statIn4 -> distronic_Enabled.distance_statIn3;
							connect v_Vehicle_msIn2 -> distronic_Enabled.v_Vehicle_msIn1;
							connect v_Obj_rel_kmhIn5 -> distronic_Enabled.v_Obj_rel_kmhIn4;
							connect v_Obj_rel_kmhIn5 -> distronic_FTS_Enabled.v_Obj_rel_kmhIn3;
							connect distance_Object_mIn3 -> distronic_Enabled.distance_Object_mIn2;
							connect distance_Object_mIn3 -> distronic_FTS_Enabled.distance_Object_mIn1;
							connect cC_active_bIn1 -> ifBlock.condition2;
						}
						component Distronic distronic;

						<<Type = "SubSystem">> component EmergencyBrake {
							port
								in Double v_Vehicle_msIn1,
								in Double distance_Object_mIn2,
								in Double v_Obj_rel_msIn3,
								out Double brakeForce_Emergency_pcOut1,
								out Double acusti_warn_bOut2;

							<<Type = "EnabledSubSystem">> component EmergencyBrake_Function {
								port
									in Double v_Vehicle_msIn1,
									in Double distance_Object_mIn2,
									in Double v_Obj_rel_msIn3,
									in Double enableIn,
									<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double brakeForce_Emergency_pcOut1,
									<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double acusti_warn_bOut2;

								<<Type = "Constant",Value = "0">> component Constant1 {
									port
										out Double out1;
								}
								component Constant1 constant1;

								<<Type = "Constant",Value = "0">> component Constant2 {
									port
										out Double out1;
								}
								component Constant2 constant2;

								<<Type = "Constant",Value = "40">> component DEMO_FAS_Emerg_Brake_Force1 {
									port
										out Double out1;
								}
								component DEMO_FAS_Emerg_Brake_Force1 dEMO_FAS_Emerg_Brake_Force1;

								<<Type = "Constant",Value = "80">> component DEMO_FAS_Emerg_Brake_Force2 {
									port
										out Double out1;
								}
								component DEMO_FAS_Emerg_Brake_Force2 dEMO_FAS_Emerg_Brake_Force2;

								<<Type = "Constant",Value = "1">> component DEMO_FAS_Emerg_Brake_Time1 {
									port
										out Double out1;
								}
								component DEMO_FAS_Emerg_Brake_Time1 dEMO_FAS_Emerg_Brake_Time1;

								<<Type = "Constant",Value = "1.5">> component DEMO_FAS_Emerg_Brake_Time12 {
									port
										out Double out1;
								}
								component DEMO_FAS_Emerg_Brake_Time12 dEMO_FAS_Emerg_Brake_Time12;

								<<Type = "Constant",Value = "8">> component DEMO_FAS_Neg_Max_ms {
									port
										out Double out1;
								}
								component DEMO_FAS_Neg_Max_ms dEMO_FAS_Neg_Max_ms;

								<<Type = "Product",Inputs = "*/">> component Div {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Div div;

								<<Type = "Product",Inputs = "*/">> component Div1 {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Div1 div1;

								<<Type = "MinMax",Function = "max">> component MinMax {
									port
										in Double in1,
										in Double in2,
										out Double brakeForce_Emergency_pcOut1;
								}
								component MinMax minMax;

								<<Type = "RelationalOperator",Operator = "<">> component RelOp {
									port
										in Double in1,
										in Double in2,
										out Double acusti_warn_bOut1;
								}
								component RelOp relOp;

								<<Type = "RelationalOperator",Operator = "<">> component RelOp1 {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component RelOp1 relOp1;

								<<Type = "Saturate",LowerLimit = "1",UpperLimit = "inf">> component Saturation1 {
									port
										in Double in1,
										out Double out1;
								}
								component Saturation1 saturation1;

								<<Type = "Saturate",LowerLimit = "1",UpperLimit = "inf">> component Saturation2 {
									port
										in Double in1,
										out Double out1;
								}
								component Saturation2 saturation2;

								<<Type = "Saturate",LowerLimit = "1",UpperLimit = "inf">> component Saturation3 {
									port
										in Double in1,
										out Double out1;
								}
								component Saturation3 saturation3;

								<<Type = "Sum",ListOfSigns = "-+">> component Sum1 {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Sum1 sum1;

								<<Type = "Sum",ListOfSigns = "-+">> component Sum2 {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component Sum2 sum2;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double out1;
								}
								component SwitchBlock switchBlock;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock1 {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double out1;
								}
								component SwitchBlock1 switchBlock1;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect saturation3.out1 -> relOp1.in1;
								connect saturation2.out1 -> relOp.in1;
								connect saturation1.out1 -> div1.in2;
								connect div1.out1 -> sum2.in2;
								connect div1.out1 -> sum1.in2;
								connect minMax.brakeForce_Emergency_pcOut1 -> brakeForce_Emergency_pcOut1;
								connect switchBlock1.out1 -> minMax.in2;
								connect switchBlock.out1 -> minMax.in1;
								connect constant2.out1 -> switchBlock1.elseIn;
								connect constant1.out1 -> switchBlock.elseIn;
								connect relOp1.out1 -> switchBlock1.condition;
								connect dEMO_FAS_Emerg_Brake_Force2.out1 -> switchBlock1.ifIn;
								connect dEMO_FAS_Emerg_Brake_Force1.out1 -> switchBlock.ifIn;
								connect relOp.acusti_warn_bOut1 -> switchBlock.condition;
								connect relOp.acusti_warn_bOut1 -> acusti_warn_bOut2;
								connect sum1.out1 -> saturation3.in1;
								connect sum2.out1 -> saturation2.in1;
								connect dEMO_FAS_Emerg_Brake_Time12.out1 -> sum1.in1;
								connect dEMO_FAS_Emerg_Brake_Time1.out1 -> sum2.in1;
								connect v_Obj_rel_msIn3 -> saturation1.in1;
								connect distance_Object_mIn2 -> div1.in1;
								connect div.out1 -> relOp1.in2;
								connect div.out1 -> relOp.in2;
								connect dEMO_FAS_Neg_Max_ms.out1 -> div.in2;
								connect v_Vehicle_msIn1 -> div.in1;
							}
							component EmergencyBrake_Function emergencyBrake_Function;

							<<Type = "SubSystem">> component VAR_TL_Constant {
								port
									out Double out1Out1;

								<<Type = "Constant",Value = "VAR_DEMO_FAS_EmergencyBrake">> component VAR_TL_Constant {
									port
										out Double out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								connect vAR_TL_Constant.out1 -> out1Out1;
							}
							component VAR_TL_Constant vAR_TL_Constant;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect vAR_TL_Constant.out1Out1 -> emergencyBrake_Function.enableIn;
							connect emergencyBrake_Function.acusti_warn_bOut2 -> acusti_warn_bOut2;
							connect emergencyBrake_Function.brakeForce_Emergency_pcOut1 -> brakeForce_Emergency_pcOut1;
							connect distance_Object_mIn2 -> emergencyBrake_Function.distance_Object_mIn2;
							connect v_Vehicle_msIn1 -> emergencyBrake_Function.v_Vehicle_msIn1;
							connect v_Obj_rel_msIn3 -> emergencyBrake_Function.v_Obj_rel_msIn3;
						}
						component EmergencyBrake emergencyBrake;

						<<Type = "SubSystem">> component FAS_Input {
							port
								in Double inputBusIn1,
								out Double fASBusOut1;

							<<Type = "BusCreator">> component BusCreator {
								port
									<<BusSignal = "ParkingBrake_b">> in Double parkingBrake_bIn1,
									<<BusSignal = "BrakeForce_pedal_pc">> in Double brakeForce_pedal_pcIn2,
									<<BusSignal = "Accelaration_pedal_pc">> in Double accelaration_pedal_pcIn3,
									<<BusSignal = "CruiseControl_b">> in Double cruiseControl_bIn4,
									<<BusSignal = "Limiter_b">> in Double limiter_bIn5,
									<<BusSignal = "LeverUp_stat">> in Double leverUp_statIn6,
									<<BusSignal = "LeverDown_stat">> in Double leverDown_statIn7,
									<<BusSignal = "V_Vehicle_kmh">> in Double v_Vehicle_kmhIn8,
									<<BusSignal = "V_Vehicle_ms">> in Double v_Vehicle_msIn9,
									<<BusSignal = "V_Sign_kmh">> in Double v_Sign_kmhIn10,
									<<BusSignal = "Sign_b">> in Double sign_bIn11,
									<<BusSignal = "Distance_stat">> in Double distance_statIn12,
									<<BusSignal = "V_Obj_rel_kmh">> in Double v_Obj_rel_kmhIn13,
									<<BusSignal = "V_Obj_rel_ms">> in Double v_Obj_rel_msIn14,
									<<BusSignal = "Distance_Object_m">> in Double distance_Object_mIn15,
									out Double fASBusOut1;
							}
							component BusCreator busCreator;

							<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
								port
									in Double in1,
									<<BusSignal = "ParkingBrake_b">> out Double parkingBrake_bOut1,
									<<BusSignal = "BrakeForce_pedal_pc">> out Double brakeForce_pedal_pcOut2,
									<<BusSignal = "Accelaration_pedal_pc">> out Double accelaration_pedal_pcOut3,
									<<BusSignal = "CruiseControl_b">> out Double cruiseControl_bOut4,
									<<BusSignal = "Limiter_b">> out Double limiter_bOut5,
									<<BusSignal = "LeverUp_stat">> out Double leverUp_statOut6,
									<<BusSignal = "LeverDown_stat">> out Double leverDown_statOut7,
									<<BusSignal = "V_Vehicle_kmh">> out Double v_Vehicle_kmhOut8,
									<<BusSignal = "V_Sign_kmh">> out Double v_Sign_kmhOut9,
									<<BusSignal = "Sign_b">> out Double sign_bOut10,
									<<BusSignal = "Distance_stat">> out Double distance_statOut11,
									<<BusSignal = "V_Obj_rel_kmh">> out Double v_Obj_rel_kmhOut12,
									<<BusSignal = "Distance_Object_m">> out Double distance_Object_mOut13;
							}
							component BusSelector busSelector;

							<<Type = "Constant",Value = "3.6">> component Constant {
								port
									out Double out1;
							}
							component Constant constant;

							<<Type = "Constant",Value = "3.6">> component Constant1 {
								port
									out Double out1;
							}
							component Constant1 constant1;

							<<Type = "Product",Inputs = "*/">> component Div {
								port
									in Double v_Vehicle_kmhIn1,
									in Double in2,
									out Double v_Vehicle_msOut1;
							}
							component Div div;

							<<Type = "Product",Inputs = "*/">> component Div1 {
								port
									in Double v_Obj_rel_kmhIn1,
									in Double in2,
									out Double v_Obj_rel_msOut1;
							}
							component Div1 div1;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect div1.v_Obj_rel_msOut1 -> busCreator.v_Obj_rel_msIn14;
							connect constant1.out1 -> div1.in2;
							connect busSelector.v_Obj_rel_kmhOut12 -> busCreator.v_Obj_rel_kmhIn13;
							connect busSelector.v_Obj_rel_kmhOut12 -> div1.v_Obj_rel_kmhIn1;
							connect div.v_Vehicle_msOut1 -> busCreator.v_Vehicle_msIn9;
							connect constant.out1 -> div.in2;
							connect busSelector.distance_statOut11 -> busCreator.distance_statIn12;
							connect busSelector.distance_Object_mOut13 -> busCreator.distance_Object_mIn15;
							connect busSelector.sign_bOut10 -> busCreator.sign_bIn11;
							connect busSelector.v_Sign_kmhOut9 -> busCreator.v_Sign_kmhIn10;
							connect busSelector.leverDown_statOut7 -> busCreator.leverDown_statIn7;
							connect busSelector.limiter_bOut5 -> busCreator.limiter_bIn5;
							connect busSelector.v_Vehicle_kmhOut8 -> busCreator.v_Vehicle_kmhIn8;
							connect busSelector.v_Vehicle_kmhOut8 -> div.v_Vehicle_kmhIn1;
							connect busSelector.brakeForce_pedal_pcOut2 -> busCreator.brakeForce_pedal_pcIn2;
							connect busSelector.accelaration_pedal_pcOut3 -> busCreator.accelaration_pedal_pcIn3;
							connect busSelector.cruiseControl_bOut4 -> busCreator.cruiseControl_bIn4;
							connect busSelector.leverUp_statOut6 -> busCreator.leverUp_statIn6;
							connect busSelector.parkingBrake_bOut1 -> busCreator.parkingBrake_bIn1;
							connect busCreator.fASBusOut1 -> fASBusOut1;
							connect inputBusIn1 -> busSelector.in1;
						}
						component FAS_Input fAS_Input;

						<<Type = "SubSystem">> component FollowToStop {
							port
								in Double v_Vehicle_kmhIn1,
								in Double v_Obj_rel_kmhIn2,
								out Double fTS_active_bOut1,
								out Double fTS_Abstand_soll_mOut2;

							<<Type = "EnabledSubSystem">> component FollowToStop_Function {
								port
									in Double v_Vehicle_kmhIn1,
									in Double v_Obj_rel_kmhIn2,
									in Double enableIn,
									<<OutputWhenDisabled = "held",InitialOutput = "[]">> out Double fTS_active_bOut1,
									<<OutputWhenDisabled = "held",InitialOutput = "[]">> out Double fTS_Abstand_soll_mOut2;

								<<Type = "SubSystem">> component Chart {
									port
										in Double v_Obj_kmhIn1,
										out Double fTS_active_bOut1,
										out Double abstand_soll_mOut2;

									<<Type = "Demux">> component Demux {
										port
											in Double in1,
											out Double out1;
									}
									component Demux demux;

									<<Type = "S-Function">> component SFunction {
										port
											in Double in1,
											out Double out1,
											out Double fTS_active_bOut2,
											out Double abstand_soll_mOut3;
									}
									component SFunction sFunction;

									<<Type = "Terminator">> component Terminator {
										port
											in Double in1;
									}
									component Terminator terminator;

									connect sFunction.out1 -> demux.in1;
									connect demux.out1 -> terminator.in1;
									connect sFunction.abstand_soll_mOut3 -> abstand_soll_mOut2;
									connect sFunction.fTS_active_bOut2 -> fTS_active_bOut1;
									connect v_Obj_kmhIn1 -> sFunction.in1;
								}
								component Chart chart;

								<<Type = "Sum",ListOfSigns = "+-">> component Sum {
									port
										in Double in1,
										in Double in2,
										out Double v_Obj_kmhOut1;
								}
								component Sum sum;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect chart.abstand_soll_mOut2 -> fTS_Abstand_soll_mOut2;
								connect chart.fTS_active_bOut1 -> fTS_active_bOut1;
								connect sum.v_Obj_kmhOut1 -> chart.v_Obj_kmhIn1;
								connect v_Vehicle_kmhIn1 -> sum.in1;
								connect v_Obj_rel_kmhIn2 -> sum.in2;
							}
							component FollowToStop_Function followToStop_Function;

							<<Type = "SubSystem">> component VAR_TL_Constant {
								port
									out Double out1Out1;

								<<Type = "Constant",Value = "VAR_DEMO_FAS_FollowToStop">> component VAR_TL_Constant {
									port
										out Double out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								connect vAR_TL_Constant.out1 -> out1Out1;
							}
							component VAR_TL_Constant vAR_TL_Constant;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect vAR_TL_Constant.out1Out1 -> followToStop_Function.enableIn;
							connect followToStop_Function.fTS_Abstand_soll_mOut2 -> fTS_Abstand_soll_mOut2;
							connect followToStop_Function.fTS_active_bOut1 -> fTS_active_bOut1;
							connect v_Obj_rel_kmhIn2 -> followToStop_Function.v_Obj_rel_kmhIn2;
							connect v_Vehicle_kmhIn1 -> followToStop_Function.v_Vehicle_kmhIn1;
						}
						component FollowToStop followToStop;

						<<Type = "SubSystem">> component Limiter {
							port
								in Double limiter_bIn1,
								in Double accelaration_pedal_pcIn2,
								in Double v_Vehicle_kmhIn3,
								in Double leverUp_statIn4,
								in Double leverDown_statIn5,
								in Double v_Sign_kmhIn6,
								in Double sign_bIn7,
								out Double vMax_kmhOut1,
								out Double limiter_active_bOut2;

							<<Type = "EnabledSubSystem">> component Limiter_Function {
								port
									in Double limiter_bIn1,
									in Double accelaration_pedal_pcIn2,
									in Double v_Vehicle_kmhIn3,
									in Double leverUp_statIn4,
									in Double leverDown_statIn5,
									in Double v_Sign_kmhIn6,
									in Double sign_bIn7,
									in Double enableIn,
									<<OutputWhenDisabled = "held",InitialOutput = "[]">> out Double vMax_kmhOut1,
									<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double limiter_active_bOut2;

								<<Type = "SubSystem">> component Limiter_Enabled {
									port
										in Double limiter_bIn1,
										in Double accelaration_pedal_pcIn2,
										out Double vMax_kmhOut1,
										out Double limiter_active_bOut2;

									<<Type = "BusCreator">> component BusCreator {
										port
											<<BusSignal = "VMax_kmh">> in Double in1,
											<<BusSignal = "Limiter_active_b">> in Double in2,
											out Double out1;
									}
									component BusCreator busCreator;

									<<Type = "BusCreator">> component BusCreator1 {
										port
											<<BusSignal = "VMax_kmh">> in Double in1,
											<<BusSignal = "Limiter_active_b">> in Double in2,
											out Double out1;
									}
									component BusCreator1 busCreator1;

									<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
										port
											in Double vMax_kmhIn1,
											<<BusSignal = "VMax_kmh">> out Double vMax_kmhOut1,
											<<BusSignal = "Limiter_active_b">> out Double limiter_active_bOut2;
									}
									component BusSelector busSelector;

									<<Type = "SubSystem">> component EdgeRising {
										port
											in Double uIn1,
											in Double rIn2,
											in Double iVIn3,
											out Double yOut1;

										<<Type = "Logic",Operator = "AND">> component LogOp_A {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component LogOp_A logOp_A;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_U {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_U memory_U;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										connect logOp_N.out1 -> logOp_A.in2;
										connect logOp_A.out1 -> yOut1;
										connect uIn1 -> logOp_A.in1;
										connect uIn1 -> memory_U.in1;
										connect switch_R.out1 -> logOp_N.in1;
										connect rIn2 -> switch_R.condition;
										connect memory_U.out1 -> switch_R.elseIn;
										connect iVIn3 -> switch_R.ifIn;
									}
									component EdgeRising edgeRising;

									<<Type = "SubSystem">> component FalseBlock {
										port
											out Double yOut1;

										<<Type = "Constant",Value = "0">> component Zero {
											port
												out Double out1;
										}
										component Zero zero;

										connect zero.out1 -> yOut1;
									}
									component FalseBlock falseBlock;

									<<Type = "If",Expression = "u2 & u1 < 90",ShowElse = "on">> component IfBlock {
										port
											in Double condition1,
											in Double condition2,
											out Double action1,
											out Double action2;
									}
									component IfBlock ifBlock;

									<<Type = "ActionSubSystem">> component Limiter_Active {
										port
											in Double ifactionIn,
											<<OutputWhenDisabled = "held",InitialOutput = "[]">> out Double vMax_kmhOut1,
											<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double limiter_active_bOut2;

										<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreRead1 {
											port
												out Double vMax_kmhOut1;
										}
										component DataStoreRead1 dataStoreRead1;

										<<Type = "Gain",Gain = "1">> component Gain {
											port
												in Double vMax_kmhIn1,
												out Double vMax_kmhOut1;
										}
										component Gain gain;

										<<Type = "Logic",Operator = "AND">> component LogicalOperator {
											port
												in Double limiter_enabled_bIn1,
												in Double limiter_enabled_bIn2,
												out Double limiter_active_bOut1;
										}
										component LogicalOperator logicalOperator;

										<<Type = "SubSystem">> component TrueBlock {
											port
												out Double yOut1;

											<<Type = "Constant",Value = "1">> component One {
												port
													out Double out1;
											}
											component One one;

											connect one.out1 -> yOut1;
										}
										component TrueBlock trueBlock;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect gain.vMax_kmhOut1 -> vMax_kmhOut1;
										connect logicalOperator.limiter_active_bOut1 -> limiter_active_bOut2;
										connect trueBlock.yOut1 -> logicalOperator.limiter_enabled_bIn1;
										connect trueBlock.yOut1 -> logicalOperator.limiter_enabled_bIn2;
										connect dataStoreRead1.vMax_kmhOut1 -> gain.vMax_kmhIn1;
									}
									component Limiter_Active limiter_Active;

									<<Type = "ActionSubSystem">> component Limiter_Deactive {
										port
											in Double ifactionIn,
											<<OutputWhenDisabled = "held",InitialOutput = "300">> out Double vMax_kmhOut1,
											<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double limiter_active_bOut2;

										<<Type = "Constant",Value = "300">> component Constant {
											port
												out Double vMax_kmhOut1;
										}
										component Constant constant;

										<<Type = "SubSystem">> component FalseBlock {
											port
												out Double yOut1;

											<<Type = "Constant",Value = "0">> component Zero {
												port
													out Double out1;
											}
											component Zero zero;

											connect zero.out1 -> yOut1;
										}
										component FalseBlock falseBlock;

										<<Type = "Gain",Gain = "1">> component Gain {
											port
												in Double vMax_kmhIn1,
												out Double vMax_kmhOut1;
										}
										component Gain gain;

										<<Type = "Logic",Operator = "AND">> component LogicalOperator {
											port
												in Double limiter_active_bIn1,
												in Double limiter_active_bIn2,
												out Double limiter_active_bOut1;
										}
										component LogicalOperator logicalOperator;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect gain.vMax_kmhOut1 -> vMax_kmhOut1;
										connect logicalOperator.limiter_active_bOut1 -> limiter_active_bOut2;
										connect constant.vMax_kmhOut1 -> gain.vMax_kmhIn1;
										connect falseBlock.yOut1 -> logicalOperator.limiter_active_bIn2;
										connect falseBlock.yOut1 -> logicalOperator.limiter_active_bIn1;
									}
									component Limiter_Deactive limiter_Deactive;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator1 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator1 logicalOperator1;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator2 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator2 logicalOperator2;

									<<Type = "Merge",InitialOutput = "[]">> component Merge {
										port
											in Double in1,
											in Double in2,
											out Double vMax_kmhOut1;
									}
									component Merge merge;

									<<Type = "SubSystem">> component RSFlipFlop {
										port
											in Double sIn1,
											in Double rIn2,
											out Double qOut1,
											out Double nOT_QOut2;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_Q {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_Q memory_Q;

										<<Type = "Constant",Value = "1">> component One_S {
											port
												out Double out1;
										}
										component One_S one_S;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_S {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_S switch_S;

										<<Type = "Constant",Value = "0">> component Zero_R {
											port
												out Double out1;
										}
										component Zero_R zero_R;

										connect memory_Q.out1 -> switch_S.elseIn;
										connect switch_S.out1 -> switch_R.elseIn;
										connect one_S.out1 -> switch_S.ifIn;
										connect sIn1 -> switch_S.condition;
										connect switch_R.out1 -> memory_Q.in1;
										connect switch_R.out1 -> qOut1;
										connect switch_R.out1 -> logOp_N.in1;
										connect zero_R.out1 -> switch_R.ifIn;
										connect rIn2 -> switch_R.condition;
										connect logOp_N.out1 -> nOT_QOut2;
									}
									component RSFlipFlop rSFlipFlop;

									<<Type = "Terminator">> component Terminator {
										port
											in Double in1;
									}
									component Terminator terminator;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect ifBlock.action2 -> limiter_Deactive.ifactionIn;
									connect accelaration_pedal_pcIn2 -> ifBlock.condition1;
									connect busSelector.limiter_active_bOut2 -> limiter_active_bOut2;
									connect busSelector.vMax_kmhOut1 -> vMax_kmhOut1;
									connect merge.vMax_kmhOut1 -> busSelector.vMax_kmhIn1;
									connect busCreator1.out1 -> merge.in2;
									connect busCreator.out1 -> merge.in1;
									connect limiter_Deactive.vMax_kmhOut1 -> busCreator1.in1;
									connect limiter_Deactive.limiter_active_bOut2 -> busCreator1.in2;
									connect limiter_Active.vMax_kmhOut1 -> busCreator.in1;
									connect limiter_Active.limiter_active_bOut2 -> busCreator.in2;
									connect logicalOperator1.out1 -> edgeRising.rIn2;
									connect falseBlock.yOut1 -> edgeRising.iVIn3;
									connect edgeRising.yOut1 -> rSFlipFlop.sIn1;
									connect limiter_bIn1 -> edgeRising.uIn1;
									connect limiter_bIn1 -> logicalOperator1.in1;
									connect limiter_bIn1 -> logicalOperator2.in1;
									connect rSFlipFlop.nOT_QOut2 -> terminator.in1;
									connect rSFlipFlop.qOut1 -> ifBlock.condition2;
									connect logicalOperator2.out1 -> rSFlipFlop.rIn2;
									connect ifBlock.action1 -> limiter_Active.ifactionIn;
								}
								component Limiter_Enabled limiter_Enabled;

								<<Type = "TriggeredSubSystem",TriggerType = "rising">> component Limiter_InitialSetValue {
									port
										in Double v_Vehicle_kmhIn1,
										in Double triggerIn;

									<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreWrite {
										port
											in Double in1;
									}
									component DataStoreWrite dataStoreWrite;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect v_Vehicle_kmhIn1 -> dataStoreWrite.in1;
								}
								component Limiter_InitialSetValue limiter_InitialSetValue;

								<<Type = "EnabledSubSystem">> component Limiter_SetValue {
									port
										in Double leverUp_statIn1,
										in Double leverDown_statIn2,
										in Double enableIn;

									<<Type = "Constant",Value = "0">> component Constant1 {
										port
											out Double out1;
									}
									component Constant1 constant1;

									<<Type = "Constant",Value = "0">> component Constant3 {
										port
											out Double out1;
									}
									component Constant3 constant3;

									<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreRead1 {
										port
											out Double v_LimiterSetValueOut1;
									}
									component DataStoreRead1 dataStoreRead1;

									<<Type = "RelationalOperator",Operator = ">">> component RelOp1 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp1 relOp1;

									<<Type = "RelationalOperator",Operator = ">">> component RelOp3 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp3 relOp3;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_LimSetValueMinus {
										port
											in Double v_LimiterSetValueIn1,
											in Double triggerIn;

										<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreWrite {
											port
												in Double in1;
										}
										component DataStoreWrite dataStoreWrite;

										<<Type = "Constant",Value = "5">> component Parameter {
											port
												out Double out1;
										}
										component Parameter parameter;

										<<Type = "Sum",ListOfSigns = "+-">> component Sum {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component Sum sum;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect sum.out1 -> dataStoreWrite.in1;
										connect parameter.out1 -> sum.in2;
										connect v_LimiterSetValueIn1 -> sum.in1;
									}
									component V_LimSetValueMinus v_LimSetValueMinus;

									<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_LimSetValuePlus {
										port
											in Double v_LimiterSetValueIn1,
											in Double triggerIn;

										<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_LimiterSetValue">> component DataStoreWrite {
											port
												in Double in1;
										}
										component DataStoreWrite dataStoreWrite;

										<<Type = "Constant",Value = "5">> component Parameter {
											port
												out Double out1;
										}
										component Parameter parameter;

										<<Type = "Sum",ListOfSigns = "++">> component Sum {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component Sum sum;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect sum.out1 -> dataStoreWrite.in1;
										connect parameter.out1 -> sum.in2;
										connect v_LimiterSetValueIn1 -> sum.in1;
									}
									component V_LimSetValuePlus v_LimSetValuePlus;

									connect relOp1.out1 -> v_LimSetValueMinus.triggerIn;
									connect constant1.out1 -> relOp1.in2;
									connect constant3.out1 -> relOp3.in2;
									connect relOp3.out1 -> v_LimSetValuePlus.triggerIn;
									connect dataStoreRead1.v_LimiterSetValueOut1 -> v_LimSetValueMinus.v_LimiterSetValueIn1;
									connect dataStoreRead1.v_LimiterSetValueOut1 -> v_LimSetValuePlus.v_LimiterSetValueIn1;
									connect leverUp_statIn1 -> relOp3.in1;
									connect leverDown_statIn2 -> relOp1.in1;
								}
								component Limiter_SetValue limiter_SetValue;

								<<Type = "TriggeredSubSystem",TriggerType = "rising">> component Limiter_StartUpSetValue {
									port
										in Double triggerIn;

									<<Type = "Constant",Value = "300">> component Constant {
										port
											out Double out1;
									}
									component Constant constant;

									<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
										port
											in Double in1;
									}
									component DataStoreWrite dataStoreWrite;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect constant.out1 -> dataStoreWrite.in1;
								}
								component Limiter_StartUpSetValue limiter_StartUpSetValue;

								<<Type = "Logic",Operator = "AND">> component LogicalOperator {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component LogicalOperator logicalOperator;

								<<Type = "MinMax",Function = "min">> component MinMax {
									port
										in Double in1,
										in Double in2,
										out Double out1;
								}
								component MinMax minMax;

								<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
									port
										in Double ifIn,
										in Double condition,
										in Double elseIn,
										out Double vMax_kmhOut1;
								}
								component SwitchBlock switchBlock;

								<<Type = "SubSystem">> component SysInit {
									port
										out Double yOut1;

									<<Type = "UnitDelay",InitialCondition = "1">> component Memory_Init {
										port
											in Double in1,
											out Double out1;
									}
									component Memory_Init memory_Init;

									<<Type = "Constant",Value = "0">> component Zero_Init {
										port
											out Double out1;
									}
									component Zero_Init zero_Init;

									connect memory_Init.out1 -> yOut1;
									connect zero_Init.out1 -> memory_Init.in1;
								}
								component SysInit sysInit;

								<<Type = "SubSystem">> component VAR_TL_Constant {
									port
										out Double out1Out1;

									<<Type = "Constant",Value = "VAR_DEMO_FAS_Signdetection">> component VAR_TL_Constant {
										port
											out Double out1;
									}
									component VAR_TL_Constant vAR_TL_Constant;

									connect vAR_TL_Constant.out1 -> out1Out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect vAR_TL_Constant.out1Out1 -> logicalOperator.in2;
								connect logicalOperator.out1 -> switchBlock.condition;
								connect v_Sign_kmhIn6 -> minMax.in1;
								connect minMax.out1 -> switchBlock.ifIn;
								connect sign_bIn7 -> logicalOperator.in1;
								connect switchBlock.vMax_kmhOut1 -> vMax_kmhOut1;
								connect sysInit.yOut1 -> limiter_StartUpSetValue.triggerIn;
								connect limiter_Enabled.limiter_active_bOut2 -> limiter_active_bOut2;
								connect accelaration_pedal_pcIn2 -> limiter_Enabled.accelaration_pedal_pcIn2;
								connect limiter_bIn1 -> limiter_Enabled.limiter_bIn1;
								connect limiter_bIn1 -> limiter_SetValue.enableIn;
								connect limiter_bIn1 -> limiter_InitialSetValue.triggerIn;
								connect limiter_Enabled.vMax_kmhOut1 -> switchBlock.elseIn;
								connect limiter_Enabled.vMax_kmhOut1 -> minMax.in2;
								connect v_Vehicle_kmhIn3 -> limiter_InitialSetValue.v_Vehicle_kmhIn1;
								connect leverUp_statIn4 -> limiter_SetValue.leverUp_statIn1;
								connect leverDown_statIn5 -> limiter_SetValue.leverDown_statIn2;
							}
							component Limiter_Function limiter_Function;

							<<Type = "SubSystem">> component VAR_TL_Constant {
								port
									out Double out1Out1;

								<<Type = "Constant",Value = "VAR_DEMO_FAS_Limiter">> component VAR_TL_Constant {
									port
										out Double out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								connect vAR_TL_Constant.out1 -> out1Out1;
							}
							component VAR_TL_Constant vAR_TL_Constant;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect vAR_TL_Constant.out1Out1 -> limiter_Function.enableIn;
							connect limiter_Function.limiter_active_bOut2 -> limiter_active_bOut2;
							connect limiter_Function.vMax_kmhOut1 -> vMax_kmhOut1;
							connect sign_bIn7 -> limiter_Function.sign_bIn7;
							connect v_Sign_kmhIn6 -> limiter_Function.v_Sign_kmhIn6;
							connect leverDown_statIn5 -> limiter_Function.leverDown_statIn5;
							connect leverUp_statIn4 -> limiter_Function.leverUp_statIn4;
							connect v_Vehicle_kmhIn3 -> limiter_Function.v_Vehicle_kmhIn3;
							connect accelaration_pedal_pcIn2 -> limiter_Function.accelaration_pedal_pcIn2;
							connect limiter_bIn1 -> limiter_Function.limiter_bIn1;
						}
						component Limiter limiter;

						<<Type = "Logic",Operator = "OR">> component LogOp {
							port
								in Double acusti_warn_bIn1,
								in Double acusti_warn_bIn2,
								out Double acusti_warn_bOut1;
						}
						component LogOp logOp;

						<<Type = "SubSystem">> component Tempomat {
							port
								in Double parkingBrake_bIn1,
								in Double brakeForce_pedal_pcIn2,
								in Double cruiseControl_bIn3,
								in Double v_Vehicle_kmhIn4,
								in Double limiter_bIn5,
								in Double leverUp_StatIn6,
								in Double leverDown_StatIn7,
								in Double fTS_active_bIn8,
								out Double v_CC_delta_kmhOut1,
								out Double cC_active_bOut2;

							<<Type = "EnabledSubSystem">> component Tempomat_Function {
								port
									in Double parkingBrake_bIn1,
									in Double brakeForce_pedal_pcIn2,
									in Double cruiseControl_bIn3,
									in Double v_Vehicle_kmhIn4,
									in Double limiter_bIn5,
									in Double leverUp_StatIn6,
									in Double leverDown_StatIn7,
									in Double fTS_active_bIn8,
									in Double enableIn,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double v_CC_delta_kmhOut1,
									<<OutputWhenDisabled = "reset",InitialOutput = "0">> out Double cC_active_bOut2;

								<<Type = "SubSystem">> component CC_Enabled {
									port
										in Double cC_active_bIn1,
										in Double v_Vehicle_kmhIn2,
										out Double v_CC_delta_kmhOut1;

									<<Type = "BusCreator">> component BusCreator {
										port
											<<BusSignal = "V_CC_delta_kmh">> in Double in1,
											out Double out1;
									}
									component BusCreator busCreator;

									<<Type = "BusCreator">> component BusCreator1 {
										port
											<<BusSignal = "V_CC_delta_kmh">> in Double in1,
											out Double out1;
									}
									component BusCreator1 busCreator1;

									<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
										port
											in Double v_CC_delta_kmhIn1,
											<<BusSignal = "V_CC_delta_kmh">> out Double v_CC_delta_kmhOut1;
									}
									component BusSelector busSelector;

									<<Type = "If",Expression = "u1",ShowElse = "on">> component IfBlock {
										port
											in Double condition1,
											out Double action1,
											out Double action2;
									}
									component IfBlock ifBlock;

									<<Type = "Merge",InitialOutput = "[]">> component Merge {
										port
											in Double in1,
											in Double in2,
											out Double v_CC_delta_kmhOut1;
									}
									component Merge merge;

									<<Type = "ActionSubSystem">> component Tempomat_Active {
										port
											in Double v_Vehicle_kmhIn1,
											in Double ifactionIn,
											<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double v_CC_delta_kmhOut1;

										<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreRead1 {
											port
												out Double out1;
										}
										component DataStoreRead1 dataStoreRead1;

										<<Type = "Sum",ListOfSigns = "-+">> component Sum {
											port
												in Double in1,
												in Double in2,
												out Double v_CC_delta_kmhOut1;
										}
										component Sum sum;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect dataStoreRead1.out1 -> sum.in2;
										connect sum.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
										connect v_Vehicle_kmhIn1 -> sum.in1;
									}
									component Tempomat_Active tempomat_Active;

									<<Type = "ActionSubSystem">> component Tempomat_Deactive {
										port
											in Double ifactionIn,
											<<OutputWhenDisabled = "held",InitialOutput = "0">> out Double v_CC_delta_kmhOut1;

										<<Type = "Constant",Value = "0">> component Constant {
											port
												out Double v_CC_delta_kmhOut1;
										}
										component Constant constant;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect constant.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
									}
									component Tempomat_Deactive tempomat_Deactive;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect ifBlock.action2 -> tempomat_Deactive.ifactionIn;
									connect cC_active_bIn1 -> ifBlock.condition1;
									connect busSelector.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
									connect merge.v_CC_delta_kmhOut1 -> busSelector.v_CC_delta_kmhIn1;
									connect busCreator1.out1 -> merge.in2;
									connect tempomat_Deactive.v_CC_delta_kmhOut1 -> busCreator1.in1;
									connect busCreator.out1 -> merge.in1;
									connect tempomat_Active.v_CC_delta_kmhOut1 -> busCreator.in1;
									connect v_Vehicle_kmhIn2 -> tempomat_Active.v_Vehicle_kmhIn1;
									connect ifBlock.action1 -> tempomat_Active.ifactionIn;
								}
								component CC_Enabled cC_Enabled;

								<<Type = "SubSystem">> component CC_On_Off {
									port
										in Double parkingBrake_bIn1,
										in Double brakeForce_pedal_pcIn2,
										in Double cruiseControl_bIn3,
										in Double v_Vehicle_kmhIn4,
										in Double limiter_bIn5,
										in Double leverUp_statIn6,
										in Double leverDown_statIn7,
										in Double fTS_active_bIn8,
										out Double cC_active_bOut1;

									<<Type = "Constant",Value = "0">> component Constant {
										port
											out Double out1;
									}
									component Constant constant;

									<<Type = "Constant",Value = "20">> component Constant1 {
										port
											out Double out1;
									}
									component Constant1 constant1;

									<<Type = "Constant",Value = "0">> component Constant2 {
										port
											out Double out1;
									}
									component Constant2 constant2;

									<<Type = "Constant",Value = "0">> component Constant3 {
										port
											out Double out1;
									}
									component Constant3 constant3;

									<<Type = "SubSystem">> component EdgeFalling {
										port
											in Double uIn1,
											in Double rIn2,
											in Double iVIn3,
											out Double yOut1;

										<<Type = "Logic",Operator = "AND">> component LogOp_A {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component LogOp_A logOp_A;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_U {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_U memory_U;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										connect switch_R.out1 -> logOp_A.in2;
										connect uIn1 -> logOp_N.in1;
										connect uIn1 -> memory_U.in1;
										connect logOp_N.out1 -> logOp_A.in1;
										connect logOp_A.out1 -> yOut1;
										connect rIn2 -> switch_R.condition;
										connect memory_U.out1 -> switch_R.elseIn;
										connect iVIn3 -> switch_R.ifIn;
									}
									component EdgeFalling edgeFalling;

									<<Type = "SubSystem">> component EdgeRising1 {
										port
											in Double uIn1,
											in Double rIn2,
											in Double iVIn3,
											out Double yOut1;

										<<Type = "Logic",Operator = "AND">> component LogOp_A {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component LogOp_A logOp_A;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_U {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_U memory_U;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										connect iVIn3 -> switch_R.ifIn;
										connect memory_U.out1 -> switch_R.elseIn;
										connect rIn2 -> switch_R.condition;
										connect switch_R.out1 -> logOp_N.in1;
										connect uIn1 -> memory_U.in1;
										connect uIn1 -> logOp_A.in1;
										connect logOp_A.out1 -> yOut1;
										connect logOp_N.out1 -> logOp_A.in2;
									}
									component EdgeRising1 edgeRising1;

									<<Type = "SubSystem">> component FalseBlock1 {
										port
											out Double yOut1;

										<<Type = "Constant",Value = "0">> component Zero {
											port
												out Double out1;
										}
										component Zero zero;

										connect zero.out1 -> yOut1;
									}
									component FalseBlock1 falseBlock1;

									<<Type = "SubSystem">> component FalseBlock2 {
										port
											out Double yOut1;

										<<Type = "Constant",Value = "0">> component Zero {
											port
												out Double out1;
										}
										component Zero zero;

										connect zero.out1 -> yOut1;
									}
									component FalseBlock2 falseBlock2;

									<<Type = "Logic",Operator = "OR">> component LogicalOperator {
										port
											in Double in1,
											in Double in2,
											in Double in3,
											in Double in4,
											out Double out1;
									}
									component LogicalOperator logicalOperator;

									<<Type = "Logic",Operator = "AND">> component LogicalOperator1 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator1 logicalOperator1;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator2 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator2 logicalOperator2;

									<<Type = "Logic",Operator = "OR">> component LogicalOperator3 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator3 logicalOperator3;

									<<Type = "Logic",Operator = "AND">> component LogicalOperator4 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator4 logicalOperator4;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator5 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator5 logicalOperator5;

									<<Type = "Logic",Operator = "OR">> component LogicalOperator6 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator6 logicalOperator6;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator7 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator7 logicalOperator7;

									<<Type = "SubSystem">> component RSFlipFlop {
										port
											in Double sIn1,
											in Double rIn2,
											out Double qOut1,
											out Double nOT_QOut2;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_Q {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_Q memory_Q;

										<<Type = "Constant",Value = "1">> component One_S {
											port
												out Double out1;
										}
										component One_S one_S;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_S {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_S switch_S;

										<<Type = "Constant",Value = "0">> component Zero_R {
											port
												out Double out1;
										}
										component Zero_R zero_R;

										connect memory_Q.out1 -> switch_S.elseIn;
										connect switch_S.out1 -> switch_R.elseIn;
										connect one_S.out1 -> switch_S.ifIn;
										connect sIn1 -> switch_S.condition;
										connect switch_R.out1 -> memory_Q.in1;
										connect switch_R.out1 -> qOut1;
										connect switch_R.out1 -> logOp_N.in1;
										connect zero_R.out1 -> switch_R.ifIn;
										connect rIn2 -> switch_R.condition;
										connect logOp_N.out1 -> nOT_QOut2;
									}
									component RSFlipFlop rSFlipFlop;

									<<Type = "RelationalOperator",Operator = ">">> component RelOp {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp relOp;

									<<Type = "RelationalOperator",Operator = "<">> component RelOp1 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp1 relOp1;

									<<Type = "RelationalOperator",Operator = ">">> component RelOp2 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp2 relOp2;

									<<Type = "RelationalOperator",Operator = ">">> component RelOp3 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp3 relOp3;

									<<Type = "Terminator">> component Terminator {
										port
											in Double in1;
									}
									component Terminator terminator;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect fTS_active_bIn8 -> logicalOperator7.in1;
									connect logicalOperator7.out1 -> logicalOperator1.in2;
									connect logicalOperator1.out1 -> logicalOperator.in4;
									connect edgeRising1.yOut1 -> logicalOperator3.in1;
									connect logicalOperator6.out1 -> edgeRising1.uIn1;
									connect logicalOperator6.out1 -> logicalOperator2.in1;
									connect falseBlock2.yOut1 -> edgeRising1.iVIn3;
									connect logicalOperator2.out1 -> edgeRising1.rIn2;
									connect logicalOperator3.out1 -> logicalOperator4.in2;
									connect constant3.out1 -> relOp3.in2;
									connect constant2.out1 -> relOp.in2;
									connect relOp.out1 -> logicalOperator6.in2;
									connect relOp3.out1 -> logicalOperator6.in1;
									connect logicalOperator5.out1 -> logicalOperator4.in1;
									connect limiter_bIn5 -> logicalOperator5.in1;
									connect cruiseControl_bIn3 -> edgeFalling.rIn2;
									connect cruiseControl_bIn3 -> edgeFalling.uIn1;
									connect cruiseControl_bIn3 -> logicalOperator3.in2;
									connect falseBlock1.yOut1 -> edgeFalling.iVIn3;
									connect edgeFalling.yOut1 -> logicalOperator.in1;
									connect v_Vehicle_kmhIn4 -> relOp1.in1;
									connect leverUp_statIn6 -> relOp3.in1;
									connect leverDown_statIn7 -> relOp.in1;
									connect relOp1.out1 -> logicalOperator1.in1;
									connect rSFlipFlop.qOut1 -> cC_active_bOut1;
									connect constant1.out1 -> relOp1.in2;
									connect brakeForce_pedal_pcIn2 -> relOp2.in1;
									connect rSFlipFlop.nOT_QOut2 -> terminator.in1;
									connect parkingBrake_bIn1 -> logicalOperator.in2;
									connect constant.out1 -> relOp2.in2;
									connect relOp2.out1 -> logicalOperator.in3;
									connect logicalOperator.out1 -> rSFlipFlop.rIn2;
									connect logicalOperator4.out1 -> rSFlipFlop.sIn1;
								}
								component CC_On_Off cC_On_Off;

								<<Type = "SubSystem">> component CC_SetValue {
									port
										in Double limiter_bIn1,
										in Double cC_enabled_bIn2,
										in Double cruiseControl_bIn3,
										in Double v_Vehicle_kmhIn4,
										in Double leverUp_StatIn5,
										in Double leverDown_StatIn6;

									<<Type = "EnabledSubSystem">> component CC_ChangeSetValue {
										port
											in Double leverUp_statIn1,
											in Double leverDown_statIn2,
											in Double enableIn;

										<<Type = "ActionSubSystem">> component CC_ChangeSetValue_Lvl2_Repeater {
											port
												in Double leverUp_StatIn1,
												in Double leverDown_StatIn2,
												in Double ifactionIn;

											<<Type = "Constant",Value = "0">> component Constant {
												port
													out Double out1;
											}
											component Constant constant;

											<<Type = "Constant",Value = "0">> component Constant1 {
												port
													out Double out1;
											}
											component Constant1 constant1;

											<<Type = "Constant",Value = "1">> component Constant2 {
												port
													out Double out1;
											}
											component Constant2 constant2;

											<<Type = "Constant",Value = "1">> component Constant3 {
												port
													out Double out1;
											}
											component Constant3 constant3;

											<<Type = "Constant",Value = "0">> component Constant4 {
												port
													out Double out1;
											}
											component Constant4 constant4;

											<<Type = "Constant",Value = "2">> component Constant5 {
												port
													out Double out1;
											}
											component Constant5 constant5;

											<<Type = "Constant",Value = "2">> component Constant6 {
												port
													out Double out1;
											}
											component Constant6 constant6;

											<<Type = "Constant",Value = "0">> component Constant7 {
												port
													out Double out1;
											}
											component Constant7 constant7;

											<<Type = "SubSystem">> component CountDown_RE {
												port
													in Double eIn1,
													in Double rIn2,
													in Double iVIn3,
													out Double yOut1;

												<<Type = "Logic",Operator = "AND">> component LogOp_C {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component LogOp_C logOp_C;

												<<Type = "UnitDelay",InitialCondition = "0">> component Memory_C {
													port
														in Double in1,
														out Double out1;
												}
												component Memory_C memory_C;

												<<Type = "Constant",Value = "1">> component One_C {
													port
														out Double out1;
												}
												component One_C one_C;

												<<Type = "RelationalOperator",Operator = ">">> component RelOp_C {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component RelOp_C relOp_C;

												<<Type = "RelationalOperator",Operator = ">">> component RelOp_Y {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component RelOp_Y relOp_Y;

												<<Type = "Sum",ListOfSigns = "+-">> component Sum_C {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum_C sum_C;

												<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_C {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component Switch_C switch_C;

												<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component Switch_R switch_R;

												<<Type = "Constant",Value = "0">> component Zero_C {
													port
														out Double out1;
												}
												component Zero_C zero_C;

												<<Type = "Constant",Value = "2.2204e-11">> component Zero_Y {
													port
														out Double out1;
												}
												component Zero_Y zero_Y;

												connect relOp_Y.out1 -> yOut1;
												connect zero_Y.out1 -> relOp_Y.in2;
												connect iVIn3 -> switch_R.ifIn;
												connect eIn1 -> logOp_C.in1;
												connect relOp_C.out1 -> logOp_C.in2;
												connect switch_R.out1 -> memory_C.in1;
												connect switch_R.out1 -> relOp_Y.in1;
												connect zero_C.out1 -> relOp_C.in2;
												connect rIn2 -> switch_R.condition;
												connect switch_C.out1 -> switch_R.elseIn;
												connect logOp_C.out1 -> switch_C.condition;
												connect one_C.out1 -> sum_C.in2;
												connect sum_C.out1 -> switch_C.ifIn;
												connect memory_C.out1 -> relOp_C.in1;
												connect memory_C.out1 -> sum_C.in1;
												connect memory_C.out1 -> switch_C.elseIn;
											}
											component CountDown_RE countDown_RE;

											<<Type = "Constant",Value = "30">> component DEMO_FAS_Repeater_Time {
												port
													out Double out1;
											}
											component DEMO_FAS_Repeater_Time dEMO_FAS_Repeater_Time;

											<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreRead1 {
												port
													out Double v_CCSetValueOut1;
											}
											component DataStoreRead1 dataStoreRead1;

											<<Type = "Logic",Operator = "NOT">> component LogicalOperator {
												port
													in Double in1,
													out Double out1;
											}
											component LogicalOperator logicalOperator;

											<<Type = "Logic",Operator = "OR">> component LogicalOperator1 {
												port
													in Double in1,
													in Double in2,
													in Double in3,
													out Double out1;
											}
											component LogicalOperator1 logicalOperator1;

											<<Type = "Logic",Operator = "NOT">> component LogicalOperator2 {
												port
													in Double in1,
													out Double out1;
											}
											component LogicalOperator2 logicalOperator2;

											<<Type = "Logic",Operator = "OR">> component LogicalOperator3 {
												port
													in Double in1,
													in Double in2,
													in Double in3,
													in Double in4,
													out Double out1;
											}
											component LogicalOperator3 logicalOperator3;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp1 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp1 relOp1;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp2 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp2 relOp2;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp3 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp3 relOp3;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp4 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp4 relOp4;

											<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
												port
													in Double ifIn,
													in Double condition,
													in Double elseIn,
													out Double out1;
											}
											component SwitchBlock switchBlock;

											<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock1 {
												port
													in Double ifIn,
													in Double condition,
													in Double elseIn,
													out Double out1;
											}
											component SwitchBlock1 switchBlock1;

											<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock2 {
												port
													in Double ifIn,
													in Double condition,
													in Double elseIn,
													out Double out1;
											}
											component SwitchBlock2 switchBlock2;

											<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock3 {
												port
													in Double ifIn,
													in Double condition,
													in Double elseIn,
													out Double out1;
											}
											component SwitchBlock3 switchBlock3;

											<<Type = "SubSystem">> component SysInit {
												port
													out Double yOut1;

												<<Type = "UnitDelay",InitialCondition = "1">> component Memory_Init {
													port
														in Double in1,
														out Double out1;
												}
												component Memory_Init memory_Init;

												<<Type = "Constant",Value = "0">> component Zero_Init {
													port
														out Double out1;
												}
												component Zero_Init zero_Init;

												connect memory_Init.out1 -> yOut1;
												connect zero_Init.out1 -> memory_Init.in1;
											}
											component SysInit sysInit;

											<<Type = "UnitDelay",InitialCondition = "0">> component UnitDelay {
												port
													in Double in1,
													out Double out1;
											}
											component UnitDelay unitDelay;

											<<Type = "SubSystem">> component VERSION_INFO {

												<<Type = "SubSystem">> component Copyright {
												}
												component Copyright copyright;
											}
											component VERSION_INFO vERSION_INFO;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValueMinus {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
													port
														in Double in1;
												}
												component DataStoreWrite dataStoreWrite;

												<<Type = "Constant",Value = "5">> component Parameter {
													port
														out Double out1;
												}
												component Parameter parameter;

												<<Type = "Sum",ListOfSigns = "+-">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect sum.out1 -> dataStoreWrite.in1;
												connect parameter.out1 -> sum.in2;
												connect v_CCSetValueIn1 -> sum.in1;
											}
											component V_SetValueMinus v_SetValueMinus;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValueMinusLvl2 {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "Constant",Value = "10">> component Constant1 {
													port
														out Double out1;
												}
												component Constant1 constant1;

												<<Type = "Constant",Value = "10">> component DEMO_FAS_CC_Lvl2_Round {
													port
														out Double out1;
												}
												component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite1 {
													port
														in Double in1;
												}
												component DataStoreWrite1 dataStoreWrite1;

												<<Type = "Math",Function = "mod">> component MathFunction {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component MathFunction mathFunction;

												<<Type = "Sum",ListOfSigns = "+-">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "DEMO_FAS_CC_Min_Value">> component SwitchBlock1 {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component SwitchBlock1 switchBlock1;

												<<Type = "Switch",Criteria = "u2 ~= 0",Threshold = "1">> component SwitchBlock3 {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component SwitchBlock3 switchBlock3;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect switchBlock3.out1 -> sum.in2;
												connect constant1.out1 -> switchBlock3.elseIn;
												connect mathFunction.out1 -> switchBlock3.ifIn;
												connect mathFunction.out1 -> switchBlock3.condition;
												connect switchBlock1.out1 -> dataStoreWrite1.in1;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
												connect v_CCSetValueIn1 -> mathFunction.in1;
												connect v_CCSetValueIn1 -> sum.in1;
												connect v_CCSetValueIn1 -> switchBlock1.elseIn;
												connect sum.out1 -> switchBlock1.condition;
												connect sum.out1 -> switchBlock1.ifIn;
											}
											component V_SetValueMinusLvl2 v_SetValueMinusLvl2;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValuePlus {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
													port
														in Double in1;
												}
												component DataStoreWrite dataStoreWrite;

												<<Type = "Constant",Value = "5">> component Parameter {
													port
														out Double out1;
												}
												component Parameter parameter;

												<<Type = "Sum",ListOfSigns = "++">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect sum.out1 -> dataStoreWrite.in1;
												connect parameter.out1 -> sum.in2;
												connect v_CCSetValueIn1 -> sum.in1;
											}
											component V_SetValuePlus v_SetValuePlus;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValuePlusLvl2 {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "Constant",Value = "10">> component DEMO_FAS_CC_Lvl2_Round {
													port
														out Double out1;
												}
												component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
													port
														in Double in1;
												}
												component DataStoreWrite dataStoreWrite;

												<<Type = "Math",Function = "mod">> component MathFunction {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component MathFunction mathFunction;

												<<Type = "Sum",ListOfSigns = "++">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "Sum",ListOfSigns = "-+">> component Sum1 {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum1 sum1;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect sum1.out1 -> sum.in2;
												connect mathFunction.out1 -> sum1.in1;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> sum1.in2;
												connect v_CCSetValueIn1 -> mathFunction.in1;
												connect v_CCSetValueIn1 -> sum.in1;
												connect sum.out1 -> dataStoreWrite.in1;
											}
											component V_SetValuePlusLvl2 v_SetValuePlusLvl2;

											connect switchBlock3.out1 -> v_SetValueMinusLvl2.triggerIn;
											connect constant7.out1 -> switchBlock3.ifIn;
											connect logicalOperator2.out1 -> unitDelay.in1;
											connect logicalOperator2.out1 -> switchBlock.condition;
											connect logicalOperator2.out1 -> switchBlock2.condition;
											connect logicalOperator2.out1 -> switchBlock3.condition;
											connect logicalOperator2.out1 -> switchBlock1.condition;
											connect relOp4.out1 -> switchBlock3.elseIn;
											connect relOp4.out1 -> logicalOperator3.in1;
											connect leverDown_StatIn2 -> relOp1.in1;
											connect leverDown_StatIn2 -> relOp4.in1;
											connect constant6.out1 -> relOp4.in2;
											connect leverUp_StatIn1 -> relOp3.in1;
											connect leverUp_StatIn1 -> relOp2.in1;
											connect constant5.out1 -> relOp2.in2;
											connect switchBlock2.out1 -> v_SetValuePlusLvl2.triggerIn;
											connect constant4.out1 -> switchBlock2.ifIn;
											connect relOp2.out1 -> switchBlock2.elseIn;
											connect relOp2.out1 -> logicalOperator3.in4;
											connect constant2.out1 -> relOp1.in2;
											connect constant3.out1 -> relOp3.in2;
											connect sysInit.yOut1 -> logicalOperator1.in3;
											connect switchBlock1.out1 -> v_SetValueMinus.triggerIn;
											connect logicalOperator3.out1 -> logicalOperator.in1;
											connect logicalOperator3.out1 -> countDown_RE.eIn1;
											connect constant1.out1 -> switchBlock1.ifIn;
											connect constant.out1 -> switchBlock.ifIn;
											connect switchBlock.out1 -> v_SetValuePlus.triggerIn;
											connect unitDelay.out1 -> logicalOperator1.in1;
											connect logicalOperator1.out1 -> countDown_RE.rIn2;
											connect logicalOperator.out1 -> logicalOperator1.in2;
											connect dEMO_FAS_Repeater_Time.out1 -> countDown_RE.iVIn3;
											connect countDown_RE.yOut1 -> logicalOperator2.in1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValuePlus.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValuePlusLvl2.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValueMinusLvl2.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValueMinus.v_CCSetValueIn1;
											connect relOp3.out1 -> switchBlock.elseIn;
											connect relOp3.out1 -> logicalOperator3.in3;
											connect relOp1.out1 -> switchBlock1.elseIn;
											connect relOp1.out1 -> logicalOperator3.in2;
										}
										component CC_ChangeSetValue_Lvl2_Repeater cC_ChangeSetValue_Lvl2_Repeater;

										<<Type = "ActionSubSystem">> component CC_ChangeSetValue_Lvl2_no_Repeater {
											port
												in Double leverUp_statIn1,
												in Double leverDown_statIn2,
												in Double ifactionIn;

											<<Type = "Constant",Value = "1">> component Constant1 {
												port
													out Double out1;
											}
											component Constant1 constant1;

											<<Type = "Constant",Value = "1">> component Constant2 {
												port
													out Double out1;
											}
											component Constant2 constant2;

											<<Type = "Constant",Value = "2">> component Constant3 {
												port
													out Double out1;
											}
											component Constant3 constant3;

											<<Type = "Constant",Value = "2">> component Constant4 {
												port
													out Double out1;
											}
											component Constant4 constant4;

											<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreRead1 {
												port
													out Double v_CCSetValueOut1;
											}
											component DataStoreRead1 dataStoreRead1;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp1 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp1 relOp1;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp2 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp2 relOp2;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp3 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp3 relOp3;

											<<Type = "RelationalOperator",Operator = "==">> component RelOp4 {
												port
													in Double in1,
													in Double in2,
													out Double out1;
											}
											component RelOp4 relOp4;

											<<Type = "SubSystem">> component VERSION_INFO {

												<<Type = "SubSystem">> component Copyright {
												}
												component Copyright copyright;
											}
											component VERSION_INFO vERSION_INFO;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValueMinus {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite1 {
													port
														in Double in1;
												}
												component DataStoreWrite1 dataStoreWrite1;

												<<Type = "Constant",Value = "5">> component Parameter {
													port
														out Double out1;
												}
												component Parameter parameter;

												<<Type = "Sum",ListOfSigns = "+-">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "DEMO_FAS_CC_Min_Value">> component SwitchBlock {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component SwitchBlock switchBlock;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect switchBlock.out1 -> dataStoreWrite1.in1;
												connect sum.out1 -> switchBlock.condition;
												connect sum.out1 -> switchBlock.ifIn;
												connect v_CCSetValueIn1 -> sum.in1;
												connect v_CCSetValueIn1 -> switchBlock.elseIn;
												connect parameter.out1 -> sum.in2;
											}
											component V_SetValueMinus v_SetValueMinus;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValueMinusLvl2 {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "Constant",Value = "10">> component Constant1 {
													port
														out Double out1;
												}
												component Constant1 constant1;

												<<Type = "Constant",Value = "10">> component DEMO_FAS_CC_Lvl2_Round {
													port
														out Double out1;
												}
												component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite1 {
													port
														in Double in1;
												}
												component DataStoreWrite1 dataStoreWrite1;

												<<Type = "Math",Function = "mod">> component MathFunction {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component MathFunction mathFunction;

												<<Type = "Sum",ListOfSigns = "+-">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "DEMO_FAS_CC_Min_Value">> component SwitchBlock1 {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component SwitchBlock1 switchBlock1;

												<<Type = "Switch",Criteria = "u2 ~= 0",Threshold = "1">> component SwitchBlock3 {
													port
														in Double ifIn,
														in Double condition,
														in Double elseIn,
														out Double out1;
												}
												component SwitchBlock3 switchBlock3;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect switchBlock3.out1 -> sum.in2;
												connect constant1.out1 -> switchBlock3.elseIn;
												connect mathFunction.out1 -> switchBlock3.ifIn;
												connect mathFunction.out1 -> switchBlock3.condition;
												connect switchBlock1.out1 -> dataStoreWrite1.in1;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
												connect v_CCSetValueIn1 -> mathFunction.in1;
												connect v_CCSetValueIn1 -> sum.in1;
												connect v_CCSetValueIn1 -> switchBlock1.elseIn;
												connect sum.out1 -> switchBlock1.condition;
												connect sum.out1 -> switchBlock1.ifIn;
											}
											component V_SetValueMinusLvl2 v_SetValueMinusLvl2;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValuePlus {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite1 {
													port
														in Double in1;
												}
												component DataStoreWrite1 dataStoreWrite1;

												<<Type = "Constant",Value = "5">> component Parameter {
													port
														out Double out1;
												}
												component Parameter parameter;

												<<Type = "Sum",ListOfSigns = "++">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect sum.out1 -> dataStoreWrite1.in1;
												connect parameter.out1 -> sum.in2;
												connect v_CCSetValueIn1 -> sum.in1;
											}
											component V_SetValuePlus v_SetValuePlus;

											<<Type = "TriggeredSubSystem",TriggerType = "rising">> component V_SetValuePlusLvl2 {
												port
													in Double v_CCSetValueIn1,
													in Double triggerIn;

												<<Type = "Constant",Value = "10">> component DEMO_FAS_CC_Lvl2_Round {
													port
														out Double out1;
												}
												component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;

												<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
													port
														in Double in1;
												}
												component DataStoreWrite dataStoreWrite;

												<<Type = "Math",Function = "mod">> component MathFunction {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component MathFunction mathFunction;

												<<Type = "Sum",ListOfSigns = "++">> component Sum {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum sum;

												<<Type = "Sum",ListOfSigns = "-+">> component Sum1 {
													port
														in Double in1,
														in Double in2,
														out Double out1;
												}
												component Sum1 sum1;

												<<Type = "SubSystem">> component VERSION_INFO {

													<<Type = "SubSystem">> component Copyright {
													}
													component Copyright copyright;
												}
												component VERSION_INFO vERSION_INFO;

												connect sum1.out1 -> sum.in2;
												connect mathFunction.out1 -> sum1.in1;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
												connect dEMO_FAS_CC_Lvl2_Round.out1 -> sum1.in2;
												connect v_CCSetValueIn1 -> mathFunction.in1;
												connect v_CCSetValueIn1 -> sum.in1;
												connect sum.out1 -> dataStoreWrite.in1;
											}
											component V_SetValuePlusLvl2 v_SetValuePlusLvl2;

											connect relOp4.out1 -> v_SetValueMinusLvl2.triggerIn;
											connect constant4.out1 -> relOp4.in2;
											connect leverDown_statIn2 -> relOp2.in1;
											connect leverDown_statIn2 -> relOp4.in1;
											connect relOp2.out1 -> v_SetValueMinus.triggerIn;
											connect constant2.out1 -> relOp2.in2;
											connect relOp1.out1 -> v_SetValuePlus.triggerIn;
											connect constant1.out1 -> relOp1.in2;
											connect constant3.out1 -> relOp3.in2;
											connect relOp3.out1 -> v_SetValuePlusLvl2.triggerIn;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValuePlusLvl2.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValuePlus.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValueMinus.v_CCSetValueIn1;
											connect dataStoreRead1.v_CCSetValueOut1 -> v_SetValueMinusLvl2.v_CCSetValueIn1;
											connect leverUp_statIn1 -> relOp3.in1;
											connect leverUp_statIn1 -> relOp1.in1;
										}
										component CC_ChangeSetValue_Lvl2_no_Repeater cC_ChangeSetValue_Lvl2_no_Repeater;

										<<Type = "If",Expression = "~u1,u1",ShowElse = "off">> component IfBlock {
											port
												in Double condition1,
												out Double action1,
												out Double action2;
										}
										component IfBlock ifBlock;

										<<Type = "SubSystem">> component VAR_TL_Constant {
											port
												out Double out1Out1;

											<<Type = "Constant",Value = "VAR_DEMO_FAS_Repeater">> component VAR_TL_Constant {
												port
													out Double out1;
											}
											component VAR_TL_Constant vAR_TL_Constant;

											connect vAR_TL_Constant.out1 -> out1Out1;
										}
										component VAR_TL_Constant vAR_TL_Constant;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect vAR_TL_Constant.out1Out1 -> ifBlock.condition1;
										connect ifBlock.action2 -> cC_ChangeSetValue_Lvl2_Repeater.ifactionIn;
										connect ifBlock.action1 -> cC_ChangeSetValue_Lvl2_no_Repeater.ifactionIn;
										connect leverDown_statIn2 -> cC_ChangeSetValue_Lvl2_no_Repeater.leverDown_statIn2;
										connect leverDown_statIn2 -> cC_ChangeSetValue_Lvl2_Repeater.leverDown_StatIn2;
										connect leverUp_statIn1 -> cC_ChangeSetValue_Lvl2_no_Repeater.leverUp_statIn1;
										connect leverUp_statIn1 -> cC_ChangeSetValue_Lvl2_Repeater.leverUp_StatIn1;
									}
									component CC_ChangeSetValue cC_ChangeSetValue;

									<<Type = "TriggeredSubSystem",TriggerType = "rising">> component CC_InitialSetValue {
										port
											in Double v_Vehicle_kmhIn1,
											in Double triggerIn;

										<<Type = "ActionSubSystem">> component CC_NoInitialSetValue {
											port
												in Double v_Vehicle_kmhIn1,
												in Double ifactionIn;

											<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
												port
													in Double in1;
											}
											component DataStoreWrite dataStoreWrite;

											<<Type = "SubSystem">> component VERSION_INFO {

												<<Type = "SubSystem">> component Copyright {
												}
												component Copyright copyright;
											}
											component VERSION_INFO vERSION_INFO;

											connect v_Vehicle_kmhIn1 -> dataStoreWrite.in1;
										}
										component CC_NoInitialSetValue cC_NoInitialSetValue;

										<<Type = "Constant",Value = "0">> component Constant {
											port
												out Double out1;
										}
										component Constant constant;

										<<Type = "DataStoreRead",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreRead1 {
											port
												out Double out1;
										}
										component DataStoreRead1 dataStoreRead1;

										<<Type = "If",Expression = "u1",ShowElse = "off">> component IfBlock {
											port
												in Double condition1,
												out Double action1;
										}
										component IfBlock ifBlock;

										<<Type = "RelationalOperator",Operator = "<">> component RelOp {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component RelOp relOp;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect ifBlock.action1 -> cC_NoInitialSetValue.ifactionIn;
										connect relOp.out1 -> ifBlock.condition1;
										connect dataStoreRead1.out1 -> relOp.in1;
										connect constant.out1 -> relOp.in2;
										connect v_Vehicle_kmhIn1 -> cC_NoInitialSetValue.v_Vehicle_kmhIn1;
									}
									component CC_InitialSetValue cC_InitialSetValue;

									<<Type = "TriggeredSubSystem",TriggerType = "rising">> component CC_OnSet_SetValue {
										port
											in Double v_Vehicle_kmhIn1,
											in Double triggerIn;

										<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
											port
												in Double in1;
										}
										component DataStoreWrite dataStoreWrite;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect v_Vehicle_kmhIn1 -> dataStoreWrite.in1;
									}
									component CC_OnSet_SetValue cC_OnSet_SetValue;

									<<Type = "TriggeredSubSystem",TriggerType = "rising">> component CC_StartUpSetValue {
										port
											in Double triggerIn;

										<<Type = "Constant",Value = "-1">> component Constant {
											port
												out Double out1;
										}
										component Constant constant;

										<<Type = "DataStoreWrite",Variable = "DEMO_FAS_V_CCSetValue">> component DataStoreWrite {
											port
												in Double in1;
										}
										component DataStoreWrite dataStoreWrite;

										<<Type = "SubSystem">> component VERSION_INFO {

											<<Type = "SubSystem">> component Copyright {
											}
											component Copyright copyright;
										}
										component VERSION_INFO vERSION_INFO;

										connect constant.out1 -> dataStoreWrite.in1;
									}
									component CC_StartUpSetValue cC_StartUpSetValue;

									<<Type = "Constant",Value = "20">> component Constant {
										port
											out Double out1;
									}
									component Constant constant;

									<<Type = "SubSystem">> component EdgeRising {
										port
											in Double uIn1,
											in Double rIn2,
											in Double iVIn3,
											out Double yOut1;

										<<Type = "Logic",Operator = "AND">> component LogOp_A {
											port
												in Double in1,
												in Double in2,
												out Double out1;
										}
										component LogOp_A logOp_A;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_U {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_U memory_U;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										connect logOp_N.out1 -> logOp_A.in2;
										connect logOp_A.out1 -> yOut1;
										connect uIn1 -> logOp_A.in1;
										connect uIn1 -> memory_U.in1;
										connect switch_R.out1 -> logOp_N.in1;
										connect rIn2 -> switch_R.condition;
										connect memory_U.out1 -> switch_R.elseIn;
										connect iVIn3 -> switch_R.ifIn;
									}
									component EdgeRising edgeRising;

									<<Type = "SubSystem">> component FalseBlock {
										port
											out Double yOut1;

										<<Type = "Constant",Value = "0">> component Zero {
											port
												out Double out1;
										}
										component Zero zero;

										connect zero.out1 -> yOut1;
									}
									component FalseBlock falseBlock;

									<<Type = "Logic",Operator = "AND">> component LogicalOperator {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator logicalOperator;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator1 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator1 logicalOperator1;

									<<Type = "Logic",Operator = "AND">> component LogicalOperator2 {
										port
											in Double in1,
											in Double in2,
											in Double in3,
											out Double out1;
									}
									component LogicalOperator2 logicalOperator2;

									<<Type = "Logic",Operator = "OR">> component LogicalOperator3 {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component LogicalOperator3 logicalOperator3;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator4 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator4 logicalOperator4;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator5 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator5 logicalOperator5;

									<<Type = "Logic",Operator = "NOT">> component LogicalOperator6 {
										port
											in Double in1,
											out Double out1;
									}
									component LogicalOperator6 logicalOperator6;

									<<Type = "SubSystem">> component RSFlipFlop {
										port
											in Double sIn1,
											in Double rIn2,
											out Double qOut1,
											out Double nOT_QOut2;

										<<Type = "Logic",Operator = "NOT">> component LogOp_N {
											port
												in Double in1,
												out Double out1;
										}
										component LogOp_N logOp_N;

										<<Type = "UnitDelay",InitialCondition = "0">> component Memory_Q {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_Q memory_Q;

										<<Type = "Constant",Value = "1">> component One_S {
											port
												out Double out1;
										}
										component One_S one_S;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_R switch_R;

										<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_S {
											port
												in Double ifIn,
												in Double condition,
												in Double elseIn,
												out Double out1;
										}
										component Switch_S switch_S;

										<<Type = "Constant",Value = "0">> component Zero_R {
											port
												out Double out1;
										}
										component Zero_R zero_R;

										connect memory_Q.out1 -> switch_S.elseIn;
										connect switch_S.out1 -> switch_R.elseIn;
										connect one_S.out1 -> switch_S.ifIn;
										connect sIn1 -> switch_S.condition;
										connect switch_R.out1 -> memory_Q.in1;
										connect switch_R.out1 -> qOut1;
										connect switch_R.out1 -> logOp_N.in1;
										connect zero_R.out1 -> switch_R.ifIn;
										connect rIn2 -> switch_R.condition;
										connect logOp_N.out1 -> nOT_QOut2;
									}
									component RSFlipFlop rSFlipFlop;

									<<Type = "RelationalOperator",Operator = ">=">> component RelOp {
										port
											in Double in1,
											in Double in2,
											out Double out1;
									}
									component RelOp relOp;

									<<Type = "SubSystem">> component SysInit {
										port
											out Double yOut1;

										<<Type = "UnitDelay",InitialCondition = "1">> component Memory_Init {
											port
												in Double in1,
												out Double out1;
										}
										component Memory_Init memory_Init;

										<<Type = "Constant",Value = "0">> component Zero_Init {
											port
												out Double out1;
										}
										component Zero_Init zero_Init;

										connect memory_Init.out1 -> yOut1;
										connect zero_Init.out1 -> memory_Init.in1;
									}
									component SysInit sysInit;

									<<Type = "Terminator">> component Terminator {
										port
											in Double in1;
									}
									component Terminator terminator;

									<<Type = "SubSystem">> component VERSION_INFO {

										<<Type = "SubSystem">> component Copyright {
										}
										component Copyright copyright;
									}
									component VERSION_INFO vERSION_INFO;

									connect rSFlipFlop.nOT_QOut2 -> terminator.in1;
									connect rSFlipFlop.qOut1 -> logicalOperator2.in3;
									connect logicalOperator6.out1 -> rSFlipFlop.rIn2;
									connect logicalOperator5.out1 -> logicalOperator2.in1;
									connect limiter_bIn1 -> logicalOperator5.in1;
									connect edgeRising.yOut1 -> rSFlipFlop.sIn1;
									connect logicalOperator4.out1 -> logicalOperator2.in2;
									connect logicalOperator2.out1 -> cC_OnSet_SetValue.triggerIn;
									connect logicalOperator1.out1 -> edgeRising.rIn2;
									connect falseBlock.yOut1 -> edgeRising.iVIn3;
									connect logicalOperator3.out1 -> edgeRising.uIn1;
									connect cC_enabled_bIn2 -> cC_ChangeSetValue.enableIn;
									connect cC_enabled_bIn2 -> logicalOperator6.in1;
									connect cC_enabled_bIn2 -> logicalOperator1.in1;
									connect leverDown_StatIn6 -> cC_ChangeSetValue.leverDown_statIn2;
									connect leverDown_StatIn6 -> logicalOperator3.in2;
									connect leverUp_StatIn5 -> cC_ChangeSetValue.leverUp_statIn1;
									connect leverUp_StatIn5 -> logicalOperator3.in1;
									connect cruiseControl_bIn3 -> logicalOperator.in1;
									connect cruiseControl_bIn3 -> logicalOperator4.in1;
									connect logicalOperator.out1 -> cC_InitialSetValue.triggerIn;
									connect relOp.out1 -> logicalOperator.in2;
									connect constant.out1 -> relOp.in2;
									connect v_Vehicle_kmhIn4 -> cC_InitialSetValue.v_Vehicle_kmhIn1;
									connect v_Vehicle_kmhIn4 -> cC_OnSet_SetValue.v_Vehicle_kmhIn1;
									connect v_Vehicle_kmhIn4 -> relOp.in1;
									connect sysInit.yOut1 -> cC_StartUpSetValue.triggerIn;
								}
								component CC_SetValue cC_SetValue;

								<<Type = "SubSystem">> component VERSION_INFO {

									<<Type = "SubSystem">> component Copyright {
									}
									component Copyright copyright;
								}
								component VERSION_INFO vERSION_INFO;

								connect fTS_active_bIn8 -> cC_On_Off.fTS_active_bIn8;
								connect limiter_bIn5 -> cC_On_Off.limiter_bIn5;
								connect limiter_bIn5 -> cC_SetValue.limiter_bIn1;
								connect cC_On_Off.cC_active_bOut1 -> cC_Enabled.cC_active_bIn1;
								connect cC_On_Off.cC_active_bOut1 -> cC_SetValue.cC_enabled_bIn2;
								connect cC_On_Off.cC_active_bOut1 -> cC_active_bOut2;
								connect brakeForce_pedal_pcIn2 -> cC_On_Off.brakeForce_pedal_pcIn2;
								connect parkingBrake_bIn1 -> cC_On_Off.parkingBrake_bIn1;
								connect cruiseControl_bIn3 -> cC_On_Off.cruiseControl_bIn3;
								connect cruiseControl_bIn3 -> cC_SetValue.cruiseControl_bIn3;
								connect v_Vehicle_kmhIn4 -> cC_Enabled.v_Vehicle_kmhIn2;
								connect v_Vehicle_kmhIn4 -> cC_SetValue.v_Vehicle_kmhIn4;
								connect v_Vehicle_kmhIn4 -> cC_On_Off.v_Vehicle_kmhIn4;
								connect cC_Enabled.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
								connect leverUp_StatIn6 -> cC_SetValue.leverUp_StatIn5;
								connect leverUp_StatIn6 -> cC_On_Off.leverUp_statIn6;
								connect leverDown_StatIn7 -> cC_SetValue.leverDown_StatIn6;
								connect leverDown_StatIn7 -> cC_On_Off.leverDown_statIn7;
							}
							component Tempomat_Function tempomat_Function;

							<<Type = "SubSystem">> component VAR_TL_Constant {
								port
									out Double out1Out1;

								<<Type = "Constant",Value = "VAR_DEMO_FAS_Tempomat">> component VAR_TL_Constant {
									port
										out Double out1;
								}
								component VAR_TL_Constant vAR_TL_Constant;

								connect vAR_TL_Constant.out1 -> out1Out1;
							}
							component VAR_TL_Constant vAR_TL_Constant;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect vAR_TL_Constant.out1Out1 -> tempomat_Function.enableIn;
							connect tempomat_Function.cC_active_bOut2 -> cC_active_bOut2;
							connect tempomat_Function.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
							connect fTS_active_bIn8 -> tempomat_Function.fTS_active_bIn8;
							connect leverDown_StatIn7 -> tempomat_Function.leverDown_StatIn7;
							connect leverUp_StatIn6 -> tempomat_Function.leverUp_StatIn6;
							connect limiter_bIn5 -> tempomat_Function.limiter_bIn5;
							connect v_Vehicle_kmhIn4 -> tempomat_Function.v_Vehicle_kmhIn4;
							connect cruiseControl_bIn3 -> tempomat_Function.cruiseControl_bIn3;
							connect brakeForce_pedal_pcIn2 -> tempomat_Function.brakeForce_pedal_pcIn2;
							connect parkingBrake_bIn1 -> tempomat_Function.parkingBrake_bIn1;
						}
						component Tempomat tempomat;

						<<Type = "SubSystem">> component VERSION_INFO {

							<<Type = "SubSystem">> component Copyright {
							}
							component Copyright copyright;
						}
						component VERSION_INFO vERSION_INFO;

						<<Type = "SubSystem">> component VelocityControl {
							port
								in Double brakeForceBoosted_pcIn1,
								in Double v_CC_delta_kmhIn2,
								in Double vMax_kmhIn3,
								in Double accelaration_pedal_pcIn4,
								in Double v_Vehicle_kmhIn5,
								in Double brakeForce_pedal_pcIn6,
								in Double brakeForce_Distronic_pcIn7,
								in Double decelerator_pcIn8,
								in Double brakeForce_Emergency_pcIn9,
								out Double acceleration_pcOut1,
								out Double brakeForce_pcOut2;

							<<Type = "Constant",Value = "0">> component Constant1 {
								port
									out Double out1;
							}
							component Constant1 constant1;

							<<Type = "Constant",Value = "0">> component Constant2 {
								port
									out Double out1;
							}
							component Constant2 constant2;

							<<Type = "Constant",Value = "0">> component Constant3 {
								port
									out Double out1;
							}
							component Constant3 constant3;

							<<Type = "Constant",Value = "0">> component Constant4 {
								port
									out Double out1;
							}
							component Constant4 constant4;

							<<Type = "Constant",Value = "100">> component Constant5 {
								port
									out Double out1;
							}
							component Constant5 constant5;

							<<Type = "Logic",Operator = "AND">> component LogOp {
								port
									in Double in1,
									in Double in2,
									out Double out1;
							}
							component LogOp logOp;

							<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[0 20 60 100]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[0 1 5 10]">> component LookUpTable {
								port
									in Double in1,
									out Double out1;
							}
							component LookUpTable lookUpTable;

							<<Type = "Lookup_n-D",NumberOfTableDimensions = "1",Table = "[100 40 5 0]",BreakpointsSpecification = "Explicit values",BreakpointsForDimension1 = "[-10 -5 -1 0]">> component LookUpTable1 {
								port
									in Double in1,
									out Double out1;
							}
							component LookUpTable1 lookUpTable1;

							<<Type = "MinMax",Function = "max">> component MinMax1 {
								port
									in Double in1,
									in Double in2,
									out Double out1;
							}
							component MinMax1 minMax1;

							<<Type = "MinMax",Function = "max">> component MinMax2 {
								port
									in Double in1,
									in Double in2,
									in Double in3,
									in Double in4,
									in Double in5,
									out Double brakeForce_pcOut1;
							}
							component MinMax2 minMax2;

							<<Type = "Product",Inputs = "**/">> component Mul {
								port
									in Double in1,
									in Double acceleration_pcIn2,
									in Double in3,
									out Double out1;
							}
							component Mul mul;

							<<Type = "RelationalOperator",Operator = ">">> component RelOp {
								port
									in Double in1,
									in Double in2,
									out Double out1;
							}
							component RelOp relOp;

							<<Type = "RelationalOperator",Operator = ">">> component RelOp1 {
								port
									in Double in1,
									in Double in2,
									out Double out1;
							}
							component RelOp1 relOp1;

							<<Type = "Saturate",LowerLimit = "0",UpperLimit = "10">> component Saturation {
								port
									in Double in1,
									out Double out1;
							}
							component Saturation saturation;

							<<Type = "Saturate",LowerLimit = "-10",UpperLimit = "0">> component Saturation1 {
								port
									in Double in1,
									out Double out1;
							}
							component Saturation1 saturation1;

							<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock {
								port
									in Double ifIn,
									in Double condition,
									in Double elseIn,
									out Double acceleration_pcOut1;
							}
							component SwitchBlock switchBlock;

							<<Type = "Switch",Criteria = "u2 > Threshold",Threshold = "0">> component SwitchBlock1 {
								port
									in Double ifIn,
									in Double condition,
									in Double elseIn,
									out Double out1;
							}
							component SwitchBlock1 switchBlock1;

							<<Type = "Switch",Criteria = "u2 > Threshold",Threshold = "1">> component SwitchBlock2 {
								port
									in Double ifIn,
									in Double condition,
									in Double elseIn,
									out Double acceleration_pcOut1;
							}
							component SwitchBlock2 switchBlock2;

							<<Type = "SubSystem">> component VERSION_INFO {

								<<Type = "SubSystem">> component Copyright {
								}
								component Copyright copyright;
							}
							component VERSION_INFO vERSION_INFO;

							connect constant5.out1 -> mul.in3;
							connect mul.out1 -> switchBlock2.elseIn;
							connect decelerator_pcIn8 -> mul.in1;
							connect constant3.out1 -> switchBlock2.ifIn;
							connect switchBlock.acceleration_pcOut1 -> mul.acceleration_pcIn2;
							connect brakeForce_Distronic_pcIn7 -> minMax2.in4;
							connect brakeForce_Emergency_pcIn9 -> minMax2.in5;
							connect constant4.out1 -> switchBlock1.ifIn;
							connect minMax1.out1 -> switchBlock.elseIn;
							connect minMax1.out1 -> switchBlock1.condition;
							connect switchBlock1.out1 -> minMax2.in1;
							connect minMax2.brakeForce_pcOut1 -> brakeForce_pcOut2;
							connect minMax2.brakeForce_pcOut1 -> switchBlock2.condition;
							connect brakeForce_pedal_pcIn6 -> minMax2.in3;
							connect lookUpTable1.out1 -> switchBlock1.elseIn;
							connect brakeForceBoosted_pcIn1 -> minMax2.in2;
							connect saturation1.out1 -> lookUpTable1.in1;
							connect saturation.out1 -> lookUpTable.in1;
							connect constant2.out1 -> relOp1.in2;
							connect switchBlock2.acceleration_pcOut1 -> acceleration_pcOut1;
							connect accelaration_pedal_pcIn4 -> minMax1.in1;
							connect vMax_kmhIn3 -> relOp.in2;
							connect vMax_kmhIn3 -> relOp1.in1;
							connect v_Vehicle_kmhIn5 -> relOp.in1;
							connect lookUpTable.out1 -> minMax1.in2;
							connect v_CC_delta_kmhIn2 -> saturation.in1;
							connect v_CC_delta_kmhIn2 -> saturation1.in1;
							connect logOp.out1 -> switchBlock.condition;
							connect relOp.out1 -> logOp.in1;
							connect relOp1.out1 -> logOp.in2;
							connect constant1.out1 -> switchBlock.ifIn;
						}
						component VelocityControl velocityControl;

						connect busSelector18.cC_active_bOut1 -> distronic.cC_active_bIn1;
						connect busSelector17.brakeForce_Distronic_pcOut1 -> velocityControl.brakeForce_Distronic_pcIn7;
						connect busSelector17.decelerator_pcOut2 -> velocityControl.decelerator_pcIn8;
						connect busCreator6.out1 -> busSelector17.in1;
						connect distronic.decelerator_pcOut2 -> busCreator6.in2;
						connect busSelector15.fTS_Abstand_soll_mOut2 -> distronic.fTS_Abstand_soll_mIn7;
						connect busCreator5.out1 -> busSelector15.in1;
						connect busCreator5.out1 -> busSelector16.in1;
						connect busSelector16.fTS_active_bOut1 -> tempomat.fTS_active_bIn8;
						connect busSelector15.fTS_active_bOut1 -> distronic.fTS_active_bIn6;
						connect followToStop.fTS_Abstand_soll_mOut2 -> busCreator5.in2;
						connect followToStop.fTS_active_bOut1 -> busCreator5.in1;
						connect busSelector14.brakeForce_Emergency_pcOut1 -> velocityControl.brakeForce_Emergency_pcIn9;
						connect busCreator4.out1 -> busSelector13.in1;
						connect busCreator4.out1 -> busSelector14.in1;
						connect busSelector13.acusti_warn_bOut1 -> logOp.acusti_warn_bIn2;
						connect busSelector12.v_Vehicle_kmhOut1 -> followToStop.v_Vehicle_kmhIn1;
						connect busSelector12.v_Obj_rel_kmhOut2 -> followToStop.v_Obj_rel_kmhIn2;
						connect logOp.acusti_warn_bOut1 -> busCreator.acusti_warn_bIn8;
						connect emergencyBrake.acusti_warn_bOut2 -> busCreator4.in2;
						connect emergencyBrake.brakeForce_Emergency_pcOut1 -> busCreator4.in1;
						connect busSelector11.v_Vehicle_msOut1 -> emergencyBrake.v_Vehicle_msIn1;
						connect busSelector11.v_Obj_rel_msOut3 -> emergencyBrake.v_Obj_rel_msIn3;
						connect busSelector11.distance_Object_mOut2 -> emergencyBrake.distance_Object_mIn2;
						connect distronic.brakeForce_Distronic_pcOut1 -> busCreator6.in1;
						connect busSelector10.v_Obj_rel_kmhOut4 -> distronic.v_Obj_rel_kmhIn5;
						connect busSelector10.distance_statOut3 -> distronic.distance_statIn4;
						connect busSelector10.v_Vehicle_msOut1 -> distronic.v_Vehicle_msIn2;
						connect busSelector10.distance_Object_mOut2 -> distronic.distance_Object_mIn3;
						connect busSelector9.acusti_warn_bOut2 -> logOp.acusti_warn_bIn1;
						connect busSelector9.opti_warn_bOut1 -> busCreator.opti_warn_bIn7;
						connect busCreator3.out1 -> busSelector9.in1;
						connect distancewarner.opti_warn_bOut1 -> busCreator3.in1;
						connect distancewarner.acusti_warn_bOut2 -> busCreator3.in2;
						connect busSelector8.distance_Object_mOut2 -> distancewarner.distance_Object_mIn2;
						connect busSelector8.v_Vehicle_msOut1 -> distancewarner.v_Vehicle_msIn1;
						connect busSelector1.v_Sign_kmhOut6 -> limiter.v_Sign_kmhIn6;
						connect busSelector1.sign_bOut7 -> limiter.sign_bIn7;
						connect velocityControl.brakeForce_pcOut2 -> busCreator.in3;
						connect busSelector2.brakeForce_pedal_pcOut3 -> velocityControl.brakeForce_pedal_pcIn6;
						connect brakeAssistant.brakeForceBoosted_pcOut1 -> velocityControl.brakeForceBoosted_pcIn1;
						connect busSelector7.brakeForce_pedal_pcOut1 -> brakeAssistant.brakeForce_pedal_pcIn1;
						connect busSelector.limiter_bOut5 -> tempomat.limiter_bIn5;
						connect busSelector6.limiter_active_bOut1 -> busCreator.limiter_active_bIn6;
						connect busCreator2.out1 -> busSelector6.in1;
						connect busCreator2.out1 -> busSelector5.in1;
						connect busSelector5.vMax_kmhOut1 -> velocityControl.vMax_kmhIn3;
						connect limiter.vMax_kmhOut1 -> busCreator2.in1;
						connect limiter.limiter_active_bOut2 -> busCreator2.in2;
						connect dataStoreRead2.limiterSetValue_kmhOut1 -> busCreator.limiterSetValue_kmhIn5;
						connect dataStoreRead1.cCSetValue_kmhOut1 -> busCreator.cCSetValue_kmhIn4;
						connect busSelector4.v_CC_delta_kmhOut1 -> velocityControl.v_CC_delta_kmhIn2;
						connect busSelector3.cC_active_bOut1 -> busCreator.cC_active_bIn1;
						connect busCreator1.out1 -> busSelector3.in1;
						connect busCreator1.out1 -> busSelector18.in1;
						connect busCreator1.out1 -> busSelector4.in1;
						connect tempomat.cC_active_bOut2 -> busCreator1.in2;
						connect tempomat.v_CC_delta_kmhOut1 -> busCreator1.in1;
						connect busSelector1.accelaration_pedal_pcOut2 -> limiter.accelaration_pedal_pcIn2;
						connect velocityControl.acceleration_pcOut1 -> busCreator.in2;
						connect busSelector2.accelaration_pedal_pcOut1 -> velocityControl.accelaration_pedal_pcIn4;
						connect busSelector2.v_Vehicle_kmhOut2 -> velocityControl.v_Vehicle_kmhIn5;
						connect busSelector1.leverUp_statOut4 -> limiter.leverUp_statIn4;
						connect busSelector1.v_Vehicle_kmhOut3 -> limiter.v_Vehicle_kmhIn3;
						connect busSelector1.limiter_bOut1 -> limiter.limiter_bIn1;
						connect busSelector1.leverDown_statOut5 -> limiter.leverDown_statIn5;
						connect busSelector.cruiseControl_bOut3 -> tempomat.cruiseControl_bIn3;
						connect busSelector.v_Vehicle_kmhOut4 -> tempomat.v_Vehicle_kmhIn4;
						connect busSelector.leverDown_statOut7 -> tempomat.leverDown_StatIn7;
						connect busSelector.leverUp_statOut6 -> tempomat.leverUp_StatIn6;
						connect busSelector.brakeForce_pedal_pcOut2 -> tempomat.brakeForce_pedal_pcIn2;
						connect busSelector.parkingBrake_bOut1 -> tempomat.parkingBrake_bIn1;
						connect fAS_Input.fASBusOut1 -> busSelector1.in1;
						connect fAS_Input.fASBusOut1 -> busSelector2.in1;
						connect fAS_Input.fASBusOut1 -> busSelector8.in1;
						connect fAS_Input.fASBusOut1 -> busSelector10.in1;
						connect fAS_Input.fASBusOut1 -> busSelector12.in1;
						connect fAS_Input.fASBusOut1 -> busSelector11.in1;
						connect fAS_Input.fASBusOut1 -> busSelector.in1;
						connect fAS_Input.fASBusOut1 -> busSelector7.in1;
						connect busCreator.outputBusOut1 -> outputBusOut1;
						connect inputBusIn1 -> fAS_Input.inputBusIn1;
					}
					component DEMO_FAS_Funktion dEMO_FAS_Funktion;

					<<Type = "SubSystem">> component DEMO_FAS_Input {
						port
							in Double parkingBrake_bIn1,
							in Double brakeForce_pedal_pcIn2,
							in Double _Accelaration_pedal_pcIn3,
							in Double cruiseControl_bIn4,
							in Double limiter_bIn5,
							in Double leverUp_statIn6,
							in Double leverDown_statIn7,
							in Double v_Vehicle_bIn8,
							in Double v_Sign_kmhIn9,
							in Double sign_bIn10,
							in Double distance_statIn11,
							in Double v_Obj_rel_kmhIn12,
							in Double distance_Object_mIn13,
							out Double inputBusOut1;

						<<Type = "BusCreator">> component BusCreator {
							port
								<<BusSignal = "ParkingBrake_b">> in Double in1,
								<<BusSignal = "BrakeForce_pedal_pc">> in Double in2,
								<<BusSignal = "Accelaration_pedal_pc">> in Double in3,
								<<BusSignal = "CruiseControl_b">> in Double in4,
								<<BusSignal = "Limiter_b">> in Double in5,
								<<BusSignal = "LeverUp_stat">> in Double in6,
								<<BusSignal = "LeverDown_stat">> in Double in7,
								<<BusSignal = "V_Vehicle_kmh">> in Double in8,
								<<BusSignal = "V_Sign_kmh">> in Double in9,
								<<BusSignal = "Sign_b">> in Double in10,
								<<BusSignal = "Distance_stat">> in Double in11,
								<<BusSignal = "V_Obj_rel_kmh">> in Double in12,
								<<BusSignal = "Distance_Object_m">> in Double in13,
								out Double inputBusOut1;
						}
						component BusCreator busCreator;

						connect distance_statIn11 -> busCreator.in11;
						connect distance_Object_mIn13 -> busCreator.in13;
						connect v_Obj_rel_kmhIn12 -> busCreator.in12;
						connect sign_bIn10 -> busCreator.in10;
						connect v_Sign_kmhIn9 -> busCreator.in9;
						connect _Accelaration_pedal_pcIn3 -> busCreator.in3;
						connect v_Vehicle_bIn8 -> busCreator.in8;
						connect busCreator.inputBusOut1 -> inputBusOut1;
						connect parkingBrake_bIn1 -> busCreator.in1;
						connect leverUp_statIn6 -> busCreator.in6;
						connect cruiseControl_bIn4 -> busCreator.in4;
						connect brakeForce_pedal_pcIn2 -> busCreator.in2;
						connect limiter_bIn5 -> busCreator.in5;
						connect leverDown_statIn7 -> busCreator.in7;
					}
					component DEMO_FAS_Input dEMO_FAS_Input;

					<<Type = "SubSystem">> component DEMO_FAS_Output {
						port
							in Double outputBusIn1,
							out Double cC_active_bOut1,
							out Double acceleration_pcOut2,
							out Double brakeForce_pcOut3,
							out Double cCSetValue_kmhOut4,
							out Double limiterSetValue_kmhOut5,
							out Double limiter_active_bOut6,
							out Double opti_warn_bOut7,
							out Double acusti_warn_bOut8;

						<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
							port
								in Double in1,
								<<BusSignal = "CC_active_b">> out Double cC_active_bOut1,
								<<BusSignal = "Acceleration_pc">> out Double acceleration_pcOut2,
								<<BusSignal = "BrakeForce_pc">> out Double brakeForce_pcOut3,
								<<BusSignal = "CCSetValue_kmh">> out Double cCSetValue_kmhOut4,
								<<BusSignal = "LimiterSetValue_kmh">> out Double limiterSetValue_kmhOut5,
								<<BusSignal = "Limiter_active_b">> out Double limiter_active_bOut6,
								<<BusSignal = "Opti_warn_b">> out Double opti_warn_bOut7,
								<<BusSignal = "Acusti_warn_b">> out Double acusti_warn_bOut8;
						}
						component BusSelector busSelector;

						connect busSelector.acusti_warn_bOut8 -> acusti_warn_bOut8;
						connect busSelector.opti_warn_bOut7 -> opti_warn_bOut7;
						connect busSelector.brakeForce_pcOut3 -> brakeForce_pcOut3;
						connect busSelector.limiter_active_bOut6 -> limiter_active_bOut6;
						connect busSelector.limiterSetValue_kmhOut5 -> limiterSetValue_kmhOut5;
						connect busSelector.cCSetValue_kmhOut4 -> cCSetValue_kmhOut4;
						connect busSelector.cC_active_bOut1 -> cC_active_bOut1;
						connect busSelector.acceleration_pcOut2 -> acceleration_pcOut2;
						connect outputBusIn1 -> busSelector.in1;
					}
					component DEMO_FAS_Output dEMO_FAS_Output;

					connect dEMO_FAS_Output.opti_warn_bOut7 -> _Opti_warn_bOut7;
					connect dEMO_FAS_Output.acusti_warn_bOut8 -> _Acusti_warn_bOut8;
					connect _Distance_statIn11 -> dEMO_FAS_Input.distance_statIn11;
					connect _Distance_Object_mIn13 -> dEMO_FAS_Input.distance_Object_mIn13;
					connect _V_Obj_rel_kmhIn12 -> dEMO_FAS_Input.v_Obj_rel_kmhIn12;
					connect _Sign_bIn10 -> dEMO_FAS_Input.sign_bIn10;
					connect _V_Sign_kmhIn9 -> dEMO_FAS_Input.v_Sign_kmhIn9;
					connect dEMO_FAS_Output.brakeForce_pcOut3 -> _BrakeForce_pcOut3;
					connect dEMO_FAS_Output.limiter_active_bOut6 -> _Limiter_active_bOut6;
					connect dEMO_FAS_Output.limiterSetValue_kmhOut5 -> _LimiterSetValue_kmhOut5;
					connect dEMO_FAS_Output.cCSetValue_kmhOut4 -> _CCSetValue_kmhOut4;
					connect dEMO_FAS_Output.cC_active_bOut1 -> _CC_active_bOut1;
					connect dEMO_FAS_Output.acceleration_pcOut2 -> _Acceleration_pcOut2;
					connect _BrakeForce_pedal_pcIn2 -> dEMO_FAS_Input.brakeForce_pedal_pcIn2;
					connect _ParkingBrake_bIn1 -> dEMO_FAS_Input.parkingBrake_bIn1;
					connect _CruiseControl_bIn4 -> dEMO_FAS_Input.cruiseControl_bIn4;
					connect _Accelaration_pedal_pcIn3 -> dEMO_FAS_Input._Accelaration_pedal_pcIn3;
					connect _Limiter_bIn5 -> dEMO_FAS_Input.limiter_bIn5;
					connect _LeverUp_statIn6 -> dEMO_FAS_Input.leverUp_statIn6;
					connect _LeverDown_statIn7 -> dEMO_FAS_Input.leverDown_statIn7;
					connect _V_Vehicle_kmhIn8 -> dEMO_FAS_Input.v_Vehicle_bIn8;
					connect dEMO_FAS_Funktion.outputBusOut1 -> dEMO_FAS_Output.outputBusIn1;
					connect dEMO_FAS_Input.inputBusOut1 -> dEMO_FAS_Funktion.inputBusIn1;
				}
				component DEMO_FAS dEMO_FAS;

				connect dEMO_FAS._Acusti_warn_bOut8 -> _Acusti_warn_bOut8;
				connect dEMO_FAS._Opti_warn_bOut7 -> _Opti_warn_bOut7;
				connect dEMO_FAS._Limiter_active_bOut6 -> _Limiter_active_bOut6;
				connect dEMO_FAS._LimiterSetValue_kmhOut5 -> _LimiterSetValue_kmhOut5;
				connect dEMO_FAS._CCSetValue_kmhOut4 -> _CCSetValue_kmhOut4;
				connect dEMO_FAS._BrakeForce_pcOut3 -> _BrakeForce_pcOut3;
				connect dEMO_FAS._Acceleration_pcOut2 -> _Acceleration_pcOut2;
				connect dEMO_FAS._CC_active_bOut1 -> _CC_active_bOut1;
				connect _Distance_Object_mIn13 -> dEMO_FAS._Distance_Object_mIn13;
				connect _V_Obj_rel_kmhIn12 -> dEMO_FAS._V_Obj_rel_kmhIn12;
				connect _Distance_statIn11 -> dEMO_FAS._Distance_statIn11;
				connect _Sign_bIn10 -> dEMO_FAS._Sign_bIn10;
				connect _V_Sign_kmhIn9 -> dEMO_FAS._V_Sign_kmhIn9;
				connect _V_Vehicle_kmhIn8 -> dEMO_FAS._V_Vehicle_kmhIn8;
				connect _LeverDown_statIn7 -> dEMO_FAS._LeverDown_statIn7;
				connect _LeverUp_statIn6 -> dEMO_FAS._LeverUp_statIn6;
				connect _Limiter_bIn5 -> dEMO_FAS._Limiter_bIn5;
				connect _CruiseControl_bIn4 -> dEMO_FAS._CruiseControl_bIn4;
				connect _Accelaration_pedal_pcIn3 -> dEMO_FAS._Accelaration_pedal_pcIn3;
				connect _BrakeForce_pedal_pcIn2 -> dEMO_FAS._BrakeForce_pedal_pcIn2;
				connect _ParkingBrake_bIn1 -> dEMO_FAS._ParkingBrake_bIn1;
			}
			component Subsystem subsystem;

			connect subsystem._Acusti_warn_bOut8 -> _Acusti_warn_bOut8;
			connect subsystem._Opti_warn_bOut7 -> _Opti_warn_bOut7;
			connect subsystem._Limiter_active_bOut6 -> _Limiter_active_bOut6;
			connect subsystem._LimiterSetValue_kmhOut5 -> _LimiterSetValue_kmhOut5;
			connect subsystem._CCSetValue_kmhOut4 -> _CCSetValue_kmhOut4;
			connect subsystem._BrakeForce_pcOut3 -> _BrakeForce_pcOut3;
			connect subsystem._Acceleration_pcOut2 -> _Acceleration_pcOut2;
			connect subsystem._CC_active_bOut1 -> _CC_active_bOut1;
			connect _Distance_Object_mIn13 -> subsystem._Distance_Object_mIn13;
			connect _V_Obj_rel_kmhIn12 -> subsystem._V_Obj_rel_kmhIn12;
			connect _Distance_statIn11 -> subsystem._Distance_statIn11;
			connect _Sign_bIn10 -> subsystem._Sign_bIn10;
			connect _V_Sign_kmhIn9 -> subsystem._V_Sign_kmhIn9;
			connect _V_Vehicle_kmhIn8 -> subsystem._V_Vehicle_kmhIn8;
			connect _LeverDown_statIn7 -> subsystem._LeverDown_statIn7;
			connect _LeverUp_statIn6 -> subsystem._LeverUp_statIn6;
			connect _Limiter_bIn5 -> subsystem._Limiter_bIn5;
			connect _CruiseControl_bIn4 -> subsystem._CruiseControl_bIn4;
			connect _Accelaration_pedal_pcIn3 -> subsystem._Accelaration_pedal_pcIn3;
			connect _BrakeForce_pedal_pcIn2 -> subsystem._BrakeForce_pedal_pcIn2;
			connect _ParkingBrake_bIn1 -> subsystem._ParkingBrake_bIn1;
		}
		component DEMO_FAS dEMO_FAS;

		connect dEMO_FAS._Acusti_warn_bOut8 -> busCreator.in8;
		connect dEMO_FAS._Opti_warn_bOut7 -> busCreator.in7;
		connect busSelector.distance_statOut11 -> dEMO_FAS._Distance_statIn11;
		connect busSelector.v_Vehicle_kmhOut8 -> dEMO_FAS._V_Vehicle_kmhIn8;
		connect busSelector.parkingBrake_bOut1 -> dEMO_FAS._ParkingBrake_bIn1;
		connect busSelector.leverUp_statOut6 -> dEMO_FAS._LeverUp_statIn6;
		connect busSelector.leverDown_statOut7 -> dEMO_FAS._LeverDown_statIn7;
		connect busSelector.brakeForce_pedal_pcOut2 -> dEMO_FAS._BrakeForce_pedal_pcIn2;
		connect busSelector.limiter_bOut5 -> dEMO_FAS._Limiter_bIn5;
		connect busSelector.cruiseControl_bOut4 -> dEMO_FAS._CruiseControl_bIn4;
		connect busSelector.accelaration_pedal_pcOut3 -> dEMO_FAS._Accelaration_pedal_pcIn3;
		connect busSelector.v_Sign_kmhOut9 -> dEMO_FAS._V_Sign_kmhIn9;
		connect busSelector.sign_bOut10 -> dEMO_FAS._Sign_bIn10;
		connect busSelector.distance_Object_mOut13 -> dEMO_FAS._Distance_Object_mIn13;
		connect busSelector.v_Obj_rel_kmhOut12 -> dEMO_FAS._V_Obj_rel_kmhIn12;
		connect dEMO_FAS._CC_active_bOut1 -> busCreator.in1;
		connect dEMO_FAS._Acceleration_pcOut2 -> busCreator.in2;
		connect dEMO_FAS._CCSetValue_kmhOut4 -> busCreator.in4;
		connect dEMO_FAS._LimiterSetValue_kmhOut5 -> busCreator.in5;
		connect dEMO_FAS._Limiter_active_bOut6 -> busCreator.in6;
		connect dEMO_FAS._BrakeForce_pcOut3 -> busCreator.in3;
		connect dEMOFAS_INIn1 -> busSelector.in1;
		connect busCreator.out1 -> dEMOFAS_OUTOut1;
	}
	component DEMO_FAS dEMO_FAS;

	<<Type = "S-Function">> component MILHandler {
	}
	component MILHandler mILHandler;

	<<Type = "Memory">> component Memory {
		port
			in Double in1,
			out Double out1;
	}
	component Memory memory;

	<<Type = "SubSystem">> component SimToRealTime {

		<<Type = "Constant",Value = "0">> component Constant {
			port
				out Double out1;
		}
		component Constant constant;

		<<Type = "Constant",Value = "1">> component Constant1 {
			port
				out Double out1;
		}
		component Constant1 constant1;

		<<Type = "Terminator">> component Terminator {
			port
				in Double in1;
		}
		component Terminator terminator;

		<<Type = "Terminator">> component Terminator1 {
			port
				in Double in1;
		}
		component Terminator1 terminator1;

		<<Type = "Terminator">> component Terminator2 {
			port
				in Double in1;
		}
		component Terminator2 terminator2;

		<<Type = "Terminator">> component Terminator3 {
			port
				in Double in1;
		}
		component Terminator3 terminator3;

		<<Type = "Terminator">> component Terminator4 {
			port
				in Double in1;
		}
		component Terminator4 terminator4;

		<<Type = "Terminator">> component Terminator5 {
			port
				in Double in1;
		}
		component Terminator5 terminator5;

		<<Type = "SubSystem">> component VAPS_TimeConfiguration {
			port
				in Double timeConfigIn1,
				in Double maxSpeedIn2,
				out Double factor_SimTimeOut1,
				out Double timeConfig_ConsumerOut2,
				out Double timeConfig_ProducerOut3,
				out Double tsSimOut4,
				out Double tsRTimeOut5,
				out Double tsRTimeEffOut6;

			<<Type = "Constant",Value = "0">> component Constant1 {
				port
					out Double out1;
			}
			component Constant1 constant1;

			<<Type = "Constant",Value = "2">> component Constant10 {
				port
					out Double out1;
			}
			component Constant10 constant10;

			<<Type = "Constant",Value = "0">> component Constant4 {
				port
					out Double out1;
			}
			component Constant4 constant4;

			<<Type = "SubSystem">> component RealTimeTimerAdapter {
				port
					in Double in1In1,
					out Double out1Out1;

				<<Type = "Constant",Value = "-10">> component Constant1 {
					port
						out Double out1;
				}
				component Constant1 constant1;

				<<Type = "Constant",Value = "1">> component Constant10 {
					port
						out Double out1;
				}
				component Constant10 constant10;

				<<Type = "Constant",Value = "1">> component Constant11 {
					port
						out Double out1;
				}
				component Constant11 constant11;

				<<Type = "Constant",Value = "-1">> component Constant12 {
					port
						out Double out1;
				}
				component Constant12 constant12;

				<<Type = "Constant",Value = "1">> component Constant13 {
					port
						out Double out1;
				}
				component Constant13 constant13;

				<<Type = "Constant",Value = "1.1">> component Constant15 {
					port
						out Double out1;
				}
				component Constant15 constant15;

				<<Type = "Product",Inputs = "*/">> component Div1 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component Div1 div1;

				<<Type = "Logic",Operator = "AND">> component LogOp1 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component LogOp1 logOp1;

				<<Type = "RelationalOperator",Operator = ">">> component RelOp2 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component RelOp2 relOp2;

				<<Type = "RelationalOperator",Operator = "<=">> component RelOp3 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component RelOp3 relOp3;

				<<Type = "RelationalOperator",Operator = ">=">> component RelOp4 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component RelOp4 relOp4;

				<<Type = "Sum",ListOfSigns = "+-">> component Sum {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component Sum sum;

				<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock2 {
					port
						in Double ifIn,
						in Double condition,
						in Double elseIn,
						out Double out1;
				}
				component SwitchBlock2 switchBlock2;

				<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock3 {
					port
						in Double ifIn,
						in Double condition,
						in Double elseIn,
						out Double out1;
				}
				component SwitchBlock3 switchBlock3;

				connect in1In1 -> relOp2.in1;
				connect in1In1 -> switchBlock2.ifIn;
				connect in1In1 -> relOp3.in1;
				connect in1In1 -> relOp4.in1;
				connect in1In1 -> div1.in1;
				connect constant10.out1 -> relOp2.in2;
				connect relOp2.out1 -> switchBlock2.condition;
				connect switchBlock2.out1 -> out1Out1;
				connect constant11.out1 -> relOp3.in2;
				connect switchBlock3.out1 -> switchBlock2.elseIn;
				connect constant12.out1 -> relOp4.in2;
				connect relOp3.out1 -> logOp1.in1;
				connect relOp4.out1 -> logOp1.in2;
				connect logOp1.out1 -> switchBlock3.condition;
				connect constant13.out1 -> switchBlock3.ifIn;
				connect sum.out1 -> switchBlock3.elseIn;
				connect constant1.out1 -> div1.in2;
				connect constant15.out1 -> sum.in1;
				connect div1.out1 -> sum.in2;
			}
			component RealTimeTimerAdapter realTimeTimerAdapter;

			<<Type = "SubSystem">> component RealTimeTimerAdapter_Value {
				port
					in Double in1In1,
					out Double out1Out1;

				<<Type = "Constant",Value = "1">> component Constant11 {
					port
						out Double out1;
				}
				component Constant11 constant11;

				<<Type = "Constant",Value = "-1">> component Constant12 {
					port
						out Double out1;
				}
				component Constant12 constant12;

				<<Type = "Constant",Value = "0">> component Constant13 {
					port
						out Double out1;
				}
				component Constant13 constant13;

				<<Type = "Logic",Operator = "AND">> component LogOp1 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component LogOp1 logOp1;

				<<Type = "RelationalOperator",Operator = "<=">> component RelOp3 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component RelOp3 relOp3;

				<<Type = "RelationalOperator",Operator = ">=">> component RelOp4 {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component RelOp4 relOp4;

				<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock3 {
					port
						in Double ifIn,
						in Double condition,
						in Double elseIn,
						out Double out1;
				}
				component SwitchBlock3 switchBlock3;

				connect constant13.out1 -> switchBlock3.ifIn;
				connect logOp1.out1 -> switchBlock3.condition;
				connect relOp4.out1 -> logOp1.in2;
				connect relOp3.out1 -> logOp1.in1;
				connect constant12.out1 -> relOp4.in2;
				connect constant11.out1 -> relOp3.in2;
				connect switchBlock3.out1 -> out1Out1;
				connect in1In1 -> switchBlock3.elseIn;
				connect in1In1 -> relOp4.in1;
				connect in1In1 -> relOp3.in1;
			}
			component RealTimeTimerAdapter_Value realTimeTimerAdapter_Value;

			<<Type = "RelationalOperator",Operator = "==">> component RelOp4 {
				port
					in Double in1,
					in Double in2,
					out Double out1;
			}
			component RelOp4 relOp4;

			<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock1 {
				port
					in Double ifIn,
					in Double condition,
					in Double elseIn,
					out Double out1;
			}
			component SwitchBlock1 switchBlock1;

			<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock2 {
				port
					in Double ifIn,
					in Double condition,
					in Double elseIn,
					out Double out1;
			}
			component SwitchBlock2 switchBlock2;

			<<Type = "SubSystem">> component SysInit1 {
				port
					out Double yOut1;

				<<Type = "UnitDelay",InitialCondition = "1">> component Memory_Init {
					port
						in Double in1,
						out Double out1;
				}
				component Memory_Init memory_Init;

				<<Type = "Constant",Value = "0">> component Zero_Init {
					port
						out Double out1;
				}
				component Zero_Init zero_Init;

				connect memory_Init.out1 -> yOut1;
				connect zero_Init.out1 -> memory_Init.in1;
			}
			component SysInit1 sysInit1;

			<<Type = "SubSystem">> component VAPSRealTimeTimer {
				port
					in Double timeFactorIn1,
					out Double tsSimOut1,
					out Double tsRTimeOut2,
					out Double tsRTimeEffectiveOut3;

				<<Type = "M-S-Function">> component Level2MfileSFunction {
					port
						in Double in1,
						out Double tsSimOut1,
						out Double tsRTimeOut2,
						out Double tsRTimeEffectiveOut3;
				}
				component Level2MfileSFunction level2MfileSFunction;

				connect timeFactorIn1 -> level2MfileSFunction.in1;
				connect level2MfileSFunction.tsSimOut1 -> tsSimOut1;
				connect level2MfileSFunction.tsRTimeOut2 -> tsRTimeOut2;
				connect level2MfileSFunction.tsRTimeEffectiveOut3 -> tsRTimeEffectiveOut3;
			}
			component VAPSRealTimeTimer vAPSRealTimeTimer;

			connect constant1.out1 -> switchBlock1.ifIn;
			connect maxSpeedIn2 -> relOp4.in1;
			connect relOp4.out1 -> switchBlock1.condition;
			connect switchBlock1.out1 -> vAPSRealTimeTimer.timeFactorIn1;
			connect constant10.out1 -> relOp4.in2;
			connect vAPSRealTimeTimer.tsRTimeEffectiveOut3 -> tsRTimeEffOut6;
			connect vAPSRealTimeTimer.tsRTimeOut2 -> tsRTimeOut5;
			connect vAPSRealTimeTimer.tsSimOut1 -> tsSimOut4;
			connect realTimeTimerAdapter_Value.out1Out1 -> timeConfig_ProducerOut3;
			connect realTimeTimerAdapter_Value.out1Out1 -> timeConfig_ConsumerOut2;
			connect realTimeTimerAdapter.out1Out1 -> switchBlock1.elseIn;
			connect realTimeTimerAdapter.out1Out1 -> factor_SimTimeOut1;
			connect timeConfigIn1 -> switchBlock2.elseIn;
			connect sysInit1.yOut1 -> switchBlock2.condition;
			connect constant4.out1 -> switchBlock2.ifIn;
			connect switchBlock2.out1 -> realTimeTimerAdapter_Value.in1In1;
			connect switchBlock2.out1 -> realTimeTimerAdapter.in1In1;
		}
		component VAPS_TimeConfiguration vAPS_TimeConfiguration;

		connect vAPS_TimeConfiguration.timeConfig_ProducerOut3 -> terminator2.in1;
		connect vAPS_TimeConfiguration.tsSimOut4 -> terminator3.in1;
		connect vAPS_TimeConfiguration.tsRTimeOut5 -> terminator4.in1;
		connect vAPS_TimeConfiguration.tsRTimeEffOut6 -> terminator5.in1;
		connect vAPS_TimeConfiguration.timeConfig_ConsumerOut2 -> terminator1.in1;
		connect vAPS_TimeConfiguration.factor_SimTimeOut1 -> terminator.in1;
		connect constant1.out1 -> vAPS_TimeConfiguration.maxSpeedIn2;
		connect constant.out1 -> vAPS_TimeConfiguration.timeConfigIn1;
	}
	component SimToRealTime simToRealTime;

	<<Type = "SubSystem">> component TargetLinkMainDialog {

		<<Type = "SubSystem">> component DSPACElogo {
		}
		component DSPACElogo dSPACElogo;
	}
	component TargetLinkMainDialog targetLinkMainDialog;

	<<Type = "SubSystem">> component Umgebung {
		port
			in Double dEMOFAS_OUTIn1,
			out Double dEMOFAS_INOut1;

		<<Type = "Display">> component Acusti_warn_b {
			port
				in Double acusti_warn_bIn1;
		}
		component Acusti_warn_b acusti_warn_b;

		<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector1 {
			port
				in Double in1,
				<<BusSignal = "CC_active_b">> out Double cC_active_bOut1,
				<<BusSignal = "CCSetValue_kmh">> out Double cCSetValue_kmhOut2,
				<<BusSignal = "LimiterSetValue_kmh">> out Double limiterSetValue_kmhOut3,
				<<BusSignal = "Limiter_active_b">> out Double limiter_active_bOut4,
				<<BusSignal = "Opti_warn_b">> out Double opti_warn_bOut5,
				<<BusSignal = "Acusti_warn_b">> out Double acusti_warn_bOut6,
				<<BusSignal = "BrakeForce_pc">> out Double brakeForce_pcOut7,
				<<BusSignal = "Acceleration_pc">> out Double acceleration_pcOut8;
		}
		component BusSelector1 busSelector1;

		<<Type = "BusCreator">> component BusCreator {
			port
				<<BusSignal = "ParkingBrake_b">> in Double parkingBrake_bIn1,
				<<BusSignal = "BrakeForce_pedal_pc">> in Double brakeForce_pedal_pcIn2,
				<<BusSignal = "Accelaration_pedal_pc">> in Double accelaration_pedal_pcIn3,
				<<BusSignal = "CruiseControl_b">> in Double cruiseControl_bIn4,
				<<BusSignal = "Limiter_b">> in Double limiter_bIn5,
				<<BusSignal = "LeverUp_stat">> in Double leverUp_statIn6,
				<<BusSignal = "LeverDown_stat">> in Double leverDown_statIn7,
				<<BusSignal = "Sign_b">> in Double sign_bIn8,
				<<BusSignal = "V_Sign_kmh">> in Double v_Sign_kmhIn9,
				<<BusSignal = "Distance_stat">> in Double distance_statIn10,
				<<BusSignal = "V_Obj_rel_kmh">> in Double in11,
				<<BusSignal = "signal12">> in Double in12,
				<<BusSignal = "V_Vehicle_kmh">> in Double v_Vehicle_kmhIn13,
				out Double dEMOFAS_INOut1;
		}
		component BusCreator busCreator;

		<<Type = "BusCreator">> component BusCreator1 {
			port
				<<BusSignal = "BrakeForce_pc">> in Double brakeForce_pcIn1,
				<<BusSignal = "Acceleration_pc">> in Double acceleration_pcIn2,
				out Double out1;
		}
		component BusCreator1 busCreator1;

		<<Type = "Display">> component CCSetValue {
			port
				in Double cCSetValue_kmhIn1;
		}
		component CCSetValue cCSetValue;

		<<Type = "Display">> component CC_active_b {
			port
				in Double cC_active_bIn1;
		}
		component CC_active_b cC_active_b;

		<<Type = "Constant",Value = "1">> component Constant {
			port
				out Double out1;
		}
		component Constant constant;

		<<Type = "Constant",Value = "1">> component Constant1 {
			port
				out Double out1;
		}
		component Constant1 constant1;

		<<Type = "Constant",Value = "1">> component Constant10 {
			port
				out Double out1;
		}
		component Constant10 constant10;

		<<Type = "Constant",Value = "2">> component Constant11 {
			port
				out Double out1;
		}
		component Constant11 constant11;

		<<Type = "Constant",Value = "1">> component Constant12 {
			port
				out Double out1;
		}
		component Constant12 constant12;

		<<Type = "Constant",Value = "1">> component Constant2 {
			port
				out Double out1;
		}
		component Constant2 constant2;

		<<Type = "Constant",Value = "1">> component Constant3 {
			port
				out Double out1;
		}
		component Constant3 constant3;

		<<Type = "Constant",Value = "1">> component Constant4 {
			port
				out Double out1;
		}
		component Constant4 constant4;

		<<Type = "Constant",Value = "400">> component Constant5 {
			port
				out Double out1;
		}
		component Constant5 constant5;

		<<Type = "Constant",Value = "1">> component Constant6 {
			port
				out Double out1;
		}
		component Constant6 constant6;

		<<Type = "Constant",Value = "1">> component Constant7 {
			port
				out Double out1;
		}
		component Constant7 constant7;

		<<Type = "Constant",Value = "0">> component Constant8 {
			port
				out Double out1;
		}
		component Constant8 constant8;

		<<Type = "Constant",Value = "0">> component Constant9 {
			port
				out Double out1;
		}
		component Constant9 constant9;

		<<Type = "DataTypeConversion">> component DataTypeConversion {
			port
				in Double distance_statIn1,
				out Double distance_statOut1;
		}
		component DataTypeConversion dataTypeConversion;

		<<Type = "Display">> component Distance_Object_m {
			port
				in Double in1;
		}
		component Distance_Object_m distance_Object_m;

		<<Type = "SubSystem">> component FalseBlock {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "0">> component Zero {
				port
					out Double out1;
			}
			component Zero zero;

			connect zero.out1 -> yOut1;
		}
		component FalseBlock falseBlock;

		<<Type = "SubSystem">> component FalseBlock1 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "0">> component Zero {
				port
					out Double out1;
			}
			component Zero zero;

			connect zero.out1 -> yOut1;
		}
		component FalseBlock1 falseBlock1;

		<<Type = "SubSystem">> component FalseBlock2 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "0">> component Zero {
				port
					out Double out1;
			}
			component Zero zero;

			connect zero.out1 -> yOut1;
		}
		component FalseBlock2 falseBlock2;

		<<Type = "SubSystem">> component FalseBlock5 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "0">> component Zero {
				port
					out Double out1;
			}
			component Zero zero;

			connect zero.out1 -> yOut1;
		}
		component FalseBlock5 falseBlock5;

		<<Type = "ManualSwitch">> component Lever_Level {
			port
				in Double in1,
				in Double in2,
				out Double out1;
		}
		component Lever_Level lever_Level;

		<<Type = "Display">> component LimiterSetValue {
			port
				in Double limiterSetValue_kmhIn1;
		}
		component LimiterSetValue limiterSetValue;

		<<Type = "Display">> component Limiter_active_b {
			port
				in Double limiter_active_bIn1;
		}
		component Limiter_active_b limiter_active_b;

		<<Type = "ManualSwitch">> component ManualSwitch {
			port
				in Double in1,
				in Double in2,
				out Double parkingBrake_bOut1;
		}
		component ManualSwitch manualSwitch;

		<<Type = "ManualSwitch">> component ManualSwitch1 {
			port
				in Double in1,
				in Double in2,
				out Double sign_bOut1;
		}
		component ManualSwitch1 manualSwitch1;

		<<Type = "ManualSwitch">> component ManualSwitch2 {
			port
				in Double in1,
				in Double in2,
				out Double cruiseControl_bOut1;
		}
		component ManualSwitch2 manualSwitch2;

		<<Type = "ManualSwitch">> component ManualSwitch5 {
			port
				in Double in1,
				in Double in2,
				out Double limiter_bOut1;
		}
		component ManualSwitch5 manualSwitch5;

		<<Type = "ManualSwitch">> component ManualSwitch6 {
			port
				in Double distance_Object_Start_mIn1,
				in Double in2,
				out Double out1;
		}
		component ManualSwitch6 manualSwitch6;

		<<Type = "ManualSwitch">> component ManualSwitch7 {
			port
				in Double in1,
				in Double in2,
				out Double out1;
		}
		component ManualSwitch7 manualSwitch7;

		<<Type = "ManualSwitch">> component ManualSwitch8 {
			port
				in Double in1,
				in Double in2,
				out Double out1;
		}
		component ManualSwitch8 manualSwitch8;

		<<Type = "Product",Inputs = "2">> component Mul {
			port
				in Double in1,
				in Double in2,
				out Double leverUp_statOut1;
		}
		component Mul mul;

		<<Type = "Product",Inputs = "2">> component Mul1 {
			port
				in Double in1,
				in Double in2,
				out Double leverDown_statOut1;
		}
		component Mul1 mul1;

		<<Type = "SubSystem">> component Object {
			port
				in Double v_Object_kmhIn1,
				in Double distance_Object_Start_mIn2,
				in Double v_Vehicle_kmhIn3,
				out Double v_Obj_rel_kmhOut1,
				out Double distance_Object_mOut2;

			<<Type = "SubSystem">> component Accumulator_REL {
				port
					in Double mXIn1,
					in Double mNIn2,
					in Double uIn3,
					in Double eIn4,
					in Double rIn5,
					in Double iVIn6,
					out Double yOut1,
					out Double b_maxOut2,
					out Double b_minOut3;

				<<Type = "SubSystem">> component Limit {
					port
						in Double mXIn1,
						in Double mNIn2,
						in Double uIn3,
						out Double yOut1,
						out Double b_maxOut2,
						out Double b_minOut3;

					<<Type = "Logic",Operator = "AND">> component LogOp_A {
						port
							in Double in1,
							in Double in2,
							out Double out1;
					}
					component LogOp_A logOp_A;

					<<Type = "Logic",Operator = "NOT">> component LogOp_N {
						port
							in Double in1,
							out Double out1;
					}
					component LogOp_N logOp_N;

					<<Type = "RelationalOperator",Operator = "<">> component RelOp_MN {
						port
							in Double in1,
							in Double in2,
							out Double out1;
					}
					component RelOp_MN relOp_MN;

					<<Type = "RelationalOperator",Operator = ">">> component RelOp_MX {
						port
							in Double in1,
							in Double in2,
							out Double out1;
					}
					component RelOp_MX relOp_MX;

					<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_MN {
						port
							in Double ifIn,
							in Double condition,
							in Double elseIn,
							out Double out1;
					}
					component Switch_MN switch_MN;

					<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_MX {
						port
							in Double ifIn,
							in Double condition,
							in Double elseIn,
							out Double out1;
					}
					component Switch_MX switch_MX;

					connect relOp_MX.out1 -> b_maxOut2;
					connect relOp_MX.out1 -> logOp_N.in1;
					connect relOp_MX.out1 -> switch_MX.condition;
					connect mXIn1 -> switch_MX.ifIn;
					connect mXIn1 -> relOp_MX.in2;
					connect mNIn2 -> relOp_MN.in2;
					connect mNIn2 -> switch_MN.ifIn;
					connect switch_MN.out1 -> switch_MX.elseIn;
					connect logOp_N.out1 -> logOp_A.in1;
					connect logOp_A.out1 -> b_minOut3;
					connect relOp_MN.out1 -> logOp_A.in2;
					connect relOp_MN.out1 -> switch_MN.condition;
					connect uIn3 -> switch_MN.elseIn;
					connect uIn3 -> relOp_MX.in1;
					connect uIn3 -> relOp_MN.in1;
					connect switch_MX.out1 -> yOut1;
				}
				component Limit limit;

				<<Type = "UnitDelay",InitialCondition = "0">> component Memory_Y {
					port
						in Double in1,
						out Double out1;
				}
				component Memory_Y memory_Y;

				<<Type = "Sum",ListOfSigns = "++">> component Sum_yu {
					port
						in Double in1,
						in Double in2,
						out Double out1;
				}
				component Sum_yu sum_yu;

				<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_E {
					port
						in Double ifIn,
						in Double condition,
						in Double elseIn,
						out Double out1;
				}
				component Switch_E switch_E;

				<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component Switch_R {
					port
						in Double ifIn,
						in Double condition,
						in Double elseIn,
						out Double out1;
				}
				component Switch_R switch_R;

				connect switch_R.out1 -> limit.uIn3;
				connect mXIn1 -> limit.mXIn1;
				connect mNIn2 -> limit.mNIn2;
				connect limit.b_minOut3 -> b_minOut3;
				connect limit.b_maxOut2 -> b_maxOut2;
				connect limit.yOut1 -> memory_Y.in1;
				connect limit.yOut1 -> yOut1;
				connect memory_Y.out1 -> switch_E.elseIn;
				connect memory_Y.out1 -> sum_yu.in2;
				connect eIn4 -> switch_E.condition;
				connect iVIn6 -> switch_R.ifIn;
				connect switch_E.out1 -> switch_R.elseIn;
				connect sum_yu.out1 -> switch_E.ifIn;
				connect uIn3 -> sum_yu.in1;
				connect rIn5 -> switch_R.condition;
			}
			component Accumulator_REL accumulator_REL;

			<<Type = "Sum",ListOfSigns = "+-">> component Add {
				port
					in Double in1,
					in Double in2,
					out Double v_Obj_rel_kmhOut1;
			}
			component Add add;

			<<Type = "Constant",Value = "300">> component Constant {
				port
					out Double out1;
			}
			component Constant constant;

			<<Type = "Constant",Value = "1000">> component Constant1 {
				port
					out Double out1;
			}
			component Constant1 constant1;

			<<Type = "Constant",Value = "0">> component Constant2 {
				port
					out Double out1;
			}
			component Constant2 constant2;

			<<Type = "Gain",Gain = "-1/3.6">> component Gain {
				port
					in Double v_Obj_rel_kmhIn1,
					out Double out1;
			}
			component Gain gain;

			<<Type = "Logic",Operator = "NOT">> component LogicalOperator {
				port
					in Double in1,
					out Double out1;
			}
			component LogicalOperator logicalOperator;

			<<Type = "RelationalOperator",Operator = ">">> component RelOp {
				port
					in Double in1,
					in Double in2,
					out Double out1;
			}
			component RelOp relOp;

			<<Type = "Terminator">> component Terminator {
				port
					in Double in1;
			}
			component Terminator terminator;

			<<Type = "Terminator">> component Terminator1 {
				port
					in Double in1;
			}
			component Terminator1 terminator1;

			connect accumulator_REL.b_minOut3 -> terminator1.in1;
			connect accumulator_REL.b_maxOut2 -> terminator.in1;
			connect logicalOperator.out1 -> accumulator_REL.eIn4;
			connect accumulator_REL.yOut1 -> distance_Object_mOut2;
			connect constant2.out1 -> accumulator_REL.mNIn2;
			connect constant1.out1 -> accumulator_REL.mXIn1;
			connect relOp.out1 -> accumulator_REL.rIn5;
			connect relOp.out1 -> logicalOperator.in1;
			connect distance_Object_Start_mIn2 -> accumulator_REL.iVIn6;
			connect distance_Object_Start_mIn2 -> relOp.in1;
			connect constant.out1 -> relOp.in2;
			connect gain.out1 -> accumulator_REL.uIn3;
			connect add.v_Obj_rel_kmhOut1 -> v_Obj_rel_kmhOut1;
			connect add.v_Obj_rel_kmhOut1 -> gain.v_Obj_rel_kmhIn1;
			connect v_Vehicle_kmhIn3 -> add.in1;
			connect v_Object_kmhIn1 -> add.in2;
		}
		component Object object;

		<<Type = "Display">> component Opti_warn_b {
			port
				in Double opti_warn_bIn1;
		}
		component Opti_warn_b opti_warn_b;

		<<Type = "SubSystem">> component SliderGain {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain sliderGain;

		<<Type = "SubSystem">> component SliderGain1 {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain1 sliderGain1;

		<<Type = "SubSystem">> component SliderGain2 {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain2 sliderGain2;

		<<Type = "SubSystem">> component SliderGain3 {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain3 sliderGain3;

		<<Type = "SubSystem">> component SliderGain4 {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain4 sliderGain4;

		<<Type = "SubSystem">> component SliderGain5 {
			port
				in Double uIn1,
				out Double yOut1;

			<<Type = "Gain",Gain = "gain">> component SliderGain {
				port
					in Double in1,
					out Double out1;
			}
			component SliderGain sliderGain;

			connect sliderGain.out1 -> yOut1;
			connect uIn1 -> sliderGain.in1;
		}
		component SliderGain5 sliderGain5;

		<<Type = "SubSystem">> component TrueBlock {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "1">> component One {
				port
					out Double out1;
			}
			component One one;

			connect one.out1 -> yOut1;
		}
		component TrueBlock trueBlock;

		<<Type = "SubSystem">> component TrueBlock1 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "1">> component One {
				port
					out Double out1;
			}
			component One one;

			connect one.out1 -> yOut1;
		}
		component TrueBlock1 trueBlock1;

		<<Type = "SubSystem">> component TrueBlock2 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "1">> component One {
				port
					out Double out1;
			}
			component One one;

			connect one.out1 -> yOut1;
		}
		component TrueBlock2 trueBlock2;

		<<Type = "SubSystem">> component TrueBlock5 {
			port
				out Double yOut1;

			<<Type = "Constant",Value = "1">> component One {
				port
					out Double out1;
			}
			component One one;

			connect one.out1 -> yOut1;
		}
		component TrueBlock5 trueBlock5;

		<<Type = "Display">> component V_Vehicle_kmh {
			port
				in Double v_Vehicle_kmhIn1;
		}
		component V_Vehicle_kmh v_Vehicle_kmh;

		<<Type = "SubSystem">> component Vehicle {
			port
				in Double busECUIn1,
				out Double v_Vehicle_kmhOut1;

			<<Type = "BusSelector",OutputAsBus = "off">> component BusSelector {
				port
					in Double in1,
					<<BusSignal = "BrakeForce_pc">> out Double brakeForce_pcOut1,
					<<BusSignal = "Acceleration_pc">> out Double acceleration_pcOut2;
			}
			component BusSelector busSelector;

			<<Type = "Constant",Value = "0">> component Constant {
				port
					out Double out1;
			}
			component Constant constant;

			<<Type = "Constant",Value = "0">> component Constant1 {
				port
					out Double out1;
			}
			component Constant1 constant1;

			<<Type = "SubSystem">> component DiscreteTransferFcnwithinitialstates {
				port
					in Double in1In1,
					out Double out1Out1;

				<<Type = "DiscreteStateSpace">> component DiscreteStateSpace {
					port
						in Double in1,
						out Double out1;
				}
				component DiscreteStateSpace discreteStateSpace;

				connect discreteStateSpace.out1 -> out1Out1;
				connect in1In1 -> discreteStateSpace.in1;
			}
			component DiscreteTransferFcnwithinitialstates discreteTransferFcnwithinitialstates;

			<<Type = "Gain",Gain = "-2">> component Gain {
				port
					in Double in1,
					out Double out1;
			}
			component Gain gain;

			<<Type = "RelationalOperator",Operator = "<">> component RelationalOperator {
				port
					in Double in1,
					in Double in2,
					out Double out1;
			}
			component RelationalOperator relationalOperator;

			<<Type = "Saturate",LowerLimit = "0",UpperLimit = "250">> component Saturation {
				port
					in Double in1,
					out Double v_Vehicle_kmhOut1;
			}
			component Saturation saturation;

			<<Type = "Sum",ListOfSigns = "+--">> component Sum1 {
				port
					in Double v_Vehicle_kmhIn1,
					in Double in2,
					in Double in3,
					out Double out1;
			}
			component Sum1 sum1;

			<<Type = "Switch",Criteria = "u2 > Threshold",Threshold = "0">> component SwitchBlock {
				port
					in Double ifIn,
					in Double condition,
					in Double elseIn,
					out Double out1;
			}
			component SwitchBlock switchBlock;

			<<Type = "Switch",Criteria = "u2 >= Threshold",Threshold = "1">> component SwitchBlock1 {
				port
					in Double ifIn,
					in Double condition,
					in Double elseIn,
					out Double out1;
			}
			component SwitchBlock1 switchBlock1;

			<<Type = "UniformRandomNumber">> component UniformRandomNumber {
				port
					out Double out1;
			}
			component UniformRandomNumber uniformRandomNumber;

			<<Type = "UnitDelay",InitialCondition = "0">> component UnitDelay {
				port
					in Double v_Vehicle_kmhIn1,
					out Double out1;
			}
			component UnitDelay unitDelay;

			<<Type = "Lookup">> component Widerstand {
				port
					in Double in1,
					out Double out1;
			}
			component Widerstand widerstand;

			connect constant1.out1 -> switchBlock1.elseIn;
			connect switchBlock1.out1 -> gain.in1;
			connect switchBlock1.out1 -> switchBlock.condition;
			connect busSelector.brakeForce_pcOut1 -> switchBlock1.ifIn;
			connect relationalOperator.out1 -> switchBlock1.condition;
			connect constant.out1 -> relationalOperator.in1;
			connect uniformRandomNumber.out1 -> sum1.in3;
			connect unitDelay.out1 -> widerstand.in1;
			connect unitDelay.out1 -> relationalOperator.in2;
			connect widerstand.out1 -> sum1.in2;
			connect discreteTransferFcnwithinitialstates.out1Out1 -> sum1.v_Vehicle_kmhIn1;
			connect sum1.out1 -> saturation.in1;
			connect busSelector.acceleration_pcOut2 -> switchBlock.elseIn;
			connect busECUIn1 -> busSelector.in1;
			connect switchBlock.out1 -> discreteTransferFcnwithinitialstates.in1In1;
			connect gain.out1 -> switchBlock.ifIn;
			connect saturation.v_Vehicle_kmhOut1 -> v_Vehicle_kmhOut1;
			connect saturation.v_Vehicle_kmhOut1 -> unitDelay.v_Vehicle_kmhIn1;
		}
		component Vehicle vehicle;

		connect busSelector1.acusti_warn_bOut6 -> acusti_warn_b.acusti_warn_bIn1;
		connect busSelector1.opti_warn_bOut5 -> opti_warn_b.opti_warn_bIn1;
		connect dataTypeConversion.distance_statOut1 -> busCreator.distance_statIn10;
		connect constant12.out1 -> sliderGain5.uIn1;
		connect sliderGain5.yOut1 -> dataTypeConversion.distance_statIn1;
		connect manualSwitch8.out1 -> mul1.in1;
		connect mul1.leverDown_statOut1 -> busCreator.leverDown_statIn7;
		connect lever_Level.out1 -> mul1.in2;
		connect lever_Level.out1 -> mul.in2;
		connect constant11.out1 -> lever_Level.in2;
		connect constant10.out1 -> lever_Level.in1;
		connect manualSwitch7.out1 -> mul.in1;
		connect mul.leverUp_statOut1 -> busCreator.leverUp_statIn6;
		connect constant9.out1 -> manualSwitch8.in2;
		connect constant8.out1 -> manualSwitch7.in2;
		connect constant7.out1 -> manualSwitch8.in1;
		connect constant6.out1 -> manualSwitch7.in1;
		connect object.distance_Object_mOut2 -> busCreator.in12;
		connect object.distance_Object_mOut2 -> distance_Object_m.in1;
		connect object.v_Obj_rel_kmhOut1 -> busCreator.in11;
		connect manualSwitch6.out1 -> object.distance_Object_Start_mIn2;
		connect constant5.out1 -> manualSwitch6.in2;
		connect sliderGain4.yOut1 -> manualSwitch6.distance_Object_Start_mIn1;
		connect constant4.out1 -> sliderGain4.uIn1;
		connect sliderGain3.yOut1 -> object.v_Object_kmhIn1;
		connect constant3.out1 -> sliderGain3.uIn1;
		connect sliderGain2.yOut1 -> busCreator.v_Sign_kmhIn9;
		connect constant2.out1 -> sliderGain2.uIn1;
		connect manualSwitch1.sign_bOut1 -> busCreator.sign_bIn8;
		connect falseBlock1.yOut1 -> manualSwitch1.in2;
		connect trueBlock1.yOut1 -> manualSwitch1.in1;
		connect busSelector1.brakeForce_pcOut7 -> busCreator1.brakeForce_pcIn1;
		connect busSelector1.limiter_active_bOut4 -> limiter_active_b.limiter_active_bIn1;
		connect busSelector1.limiterSetValue_kmhOut3 -> limiterSetValue.limiterSetValue_kmhIn1;
		connect busSelector1.cCSetValue_kmhOut2 -> cCSetValue.cCSetValue_kmhIn1;
		connect busSelector1.cC_active_bOut1 -> cC_active_b.cC_active_bIn1;
		connect constant1.out1 -> sliderGain1.uIn1;
		connect sliderGain.yOut1 -> busCreator.accelaration_pedal_pcIn3;
		connect constant.out1 -> sliderGain.uIn1;
		connect busSelector1.acceleration_pcOut8 -> busCreator1.acceleration_pcIn2;
		connect dEMOFAS_OUTIn1 -> busSelector1.in1;
		connect busCreator1.out1 -> vehicle.busECUIn1;
		connect vehicle.v_Vehicle_kmhOut1 -> v_Vehicle_kmh.v_Vehicle_kmhIn1;
		connect vehicle.v_Vehicle_kmhOut1 -> busCreator.v_Vehicle_kmhIn13;
		connect vehicle.v_Vehicle_kmhOut1 -> object.v_Vehicle_kmhIn3;
		connect manualSwitch5.limiter_bOut1 -> busCreator.limiter_bIn5;
		connect falseBlock5.yOut1 -> manualSwitch5.in2;
		connect trueBlock5.yOut1 -> manualSwitch5.in1;
		connect manualSwitch2.cruiseControl_bOut1 -> busCreator.cruiseControl_bIn4;
		connect falseBlock2.yOut1 -> manualSwitch2.in2;
		connect trueBlock2.yOut1 -> manualSwitch2.in1;
		connect sliderGain1.yOut1 -> busCreator.brakeForce_pedal_pcIn2;
		connect manualSwitch.parkingBrake_bOut1 -> busCreator.parkingBrake_bIn1;
		connect falseBlock.yOut1 -> manualSwitch.in2;
		connect trueBlock.yOut1 -> manualSwitch.in1;
		connect busCreator.dEMOFAS_INOut1 -> dEMOFAS_INOut1;
	}
	component Umgebung umgebung;

	connect umgebung.dEMOFAS_INOut1 -> dEMO_FAS.dEMOFAS_INIn1;
	connect memory.out1 -> umgebung.dEMOFAS_OUTIn1;
	connect dEMO_FAS.dEMOFAS_OUTOut1 -> memory.in1;
}
