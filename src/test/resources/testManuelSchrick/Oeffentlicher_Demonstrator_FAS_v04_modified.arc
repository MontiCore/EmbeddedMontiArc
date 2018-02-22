package testManuelSchrick;
import java.lang.*;
import java.util.*;
<<Type="SubSystem">> component Oeffentlicher_Demonstrator_FAS_v04_modified {
  <<Type="SubSystem">> component DEMO_FAS {
    ports
      out Boolean cC_active_b,
      out Double cCSetValue_kmh,
      out Double limiterSetValue_kmh,
      out Boolean limiter_active_b,
      out Boolean opti_warn_b,
      out Boolean acusti_warn_b,
      out Double brakeForce_pc,
      out Double acceleration_pc,
      in Boolean parkingBrake_b,
      in Double brakeForce_pedal_pc,
      in Double acceleration_pedal_pc,
      in Boolean cruiseControl_b,
      in Boolean limiter_b,
      in Double leverUp_stat,
      in Double leverDown_stat,
      in Double v_Vehicle_kmh,
      in Double v_Sign_kmh,
      in Boolean sign_b,
      in Double distance_stat,
      in Double v_Obj_rel_kmh;

      component Cons0 {
        ports
           out Double out1;
       }

      component Cons0 cons0;

      connect cons0.out1 -> dEMO_FAS._Distance_Object_mIn13;

    <<Type="SubSystem">> component DEMO_FAS {
      ports
        in Boolean _ParkingBrake_bIn1,
        in Double _BrakeForce_pedal_pcIn2,
        in Double _Acceleration_pedal_pcIn3,
        in Boolean _CruiseControl_bIn4,
        in Boolean _Limiter_bIn5,
        in Double _LeverUp_statIn6,
        in Double _LeverDown_statIn7,
        in Double _V_Vehicle_kmhIn8,
        in Double _V_Sign_kmhIn9,
        in Boolean _Sign_bIn10,
        in Double _Distance_statIn11,
        in Double _V_Obj_rel_kmhIn12,
        in Double _Distance_Object_mIn13,
        out Boolean _CC_active_bOut1,
        out Double _Acceleration_pcOut2,
        out Double _BrakeForce_pcOut3,
        out Double _CCSetValue_kmhOut4,
        out Double _LimiterSetValue_kmhOut5,
        out Boolean _Limiter_active_bOut6,
        out Boolean _Opti_warn_bOut7,
        out Boolean _Acusti_warn_bOut8;
      <<Type="SubSystem">> component Subsystem {
        ports
          in Boolean _ParkingBrake_bIn1,
          in Double _BrakeForce_pedal_pcIn2,
          in Double _Acceleration_pedal_pcIn3,
          in Boolean _CruiseControl_bIn4,
          in Boolean _Limiter_bIn5,
          in Double _LeverUp_statIn6,
          in Double _LeverDown_statIn7,
          in Double _V_Vehicle_kmhIn8,
          in Double _V_Sign_kmhIn9,
          in Boolean _Sign_bIn10,
          in Double _Distance_statIn11,
          in Double _V_Obj_rel_kmhIn12,
          in Double _Distance_Object_mIn13,
          out Boolean _CC_active_bOut1,
          out Double _Acceleration_pcOut2,
          out Double _BrakeForce_pcOut3,
          out Double _CCSetValue_kmhOut4,
          out Double _LimiterSetValue_kmhOut5,
          out Boolean _Limiter_active_bOut6,
          out Boolean _Opti_warn_bOut7,
          out Boolean _Acusti_warn_bOut8;
        <<Type="SubSystem">> component DEMO_FAS {
          ports
            in Boolean _ParkingBrake_bIn1,
            in Double _BrakeForce_pedal_pcIn2,
            in Double _Acceleration_pedal_pcIn3,
            in Boolean _CruiseControl_bIn4,
            in Boolean _Limiter_bIn5,
            in Double _LeverUp_statIn6,
            in Double _LeverDown_statIn7,
            in Double _V_Vehicle_kmhIn8,
            in Double _V_Sign_kmhIn9,
            in Boolean _Sign_bIn10,
            in Double _Distance_statIn11,
            in Double _V_Obj_rel_kmhIn12,
            in Double _Distance_Object_mIn13,
            out Boolean _CC_active_bOut1,
            out Double _Acceleration_pcOut2,
            out Double _BrakeForce_pcOut3,
            out Double _CCSetValue_kmhOut4,
            out Double _LimiterSetValue_kmhOut5,
            out Boolean _Limiter_active_bOut6,
            out Boolean _Opti_warn_bOut7,
            out Boolean _Acusti_warn_bOut8;
          <<Type="SubSystem">> component DEMO_FAS_Funktion {
            ports
              in Boolean parkingBrake_b,
              in Double brakeForce_pedal_pc,
              in Double acceleration_pedal_pc,
              in Boolean cruiseControl_b,
              in Boolean limiter_b,
              in Double leverUp_stat,
              in Double leverDown_stat,
              in Double v_Vehicle_kmh,
              in Double v_Vehicle_kmh1,
              in Double v_Sign_kmh,
              in Boolean sign_b,
              in Double distance_stat,
              in Double distance_stat1,
              in Double v_Obj_rel_kmh,
              in Double v_Obj_rel_kmh1,
              in Double v_Obj_rel_kmh2,
              in Double distance_Object_m,
              in Double distance_Object_m1,
              in Double distance_Object_m2,
              in Double distance_Object_m3,
              out Boolean cC_active_b,
              out Double acceleration_pc,
              out Double brakeForce_pc,
              out Double cCSetValue_kmh,
              out Double limiterSetValue_kmh,
              out Boolean limiter_active_b,
              out Boolean opti_warn_b,
              out Boolean acusti_warn_b;
            <<Type="SubSystem">> component BrakeAssistant {
              ports
                in Double brakeForce_pedal_pcIn1,
                out Double brakeForceBoosted_pcOut1;
              <<Type="SubSystem">> component Brake_Booster {
                ports
                  in Double brakeForce_pedal_pcIn1,
                  out Double brakeForceBoosted_pcOut1;
                <<Type="Constant",Value="100">> component Constant {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="70">> component DEMO_FAS_BrakeBooster_Threshold {
                  ports
                    out Double out1;
                }
                <<Operator=">=",Type="RelationalOperator">> component RelOp {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double brakeForceBoosted_pcOut1;
                  effect ifIn -> brakeForceBoosted_pcOut1;
                  effect condition -> brakeForceBoosted_pcOut1;
                  effect elseIn -> brakeForceBoosted_pcOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                component Constant constant;
                component DEMO_FAS_BrakeBooster_Threshold dEMO_FAS_BrakeBooster_Threshold;
                component RelOp relOp;
                component SwitchBlock switchBlock;
                component VERSION_INFO vERSION_INFO;
                component Condition condition;
                connect condition.out1 -> switchBlock.condition;
                connect relOp.out1 -> condition.in1;
                connect switchBlock.brakeForceBoosted_pcOut1 -> brakeForceBoosted_pcOut1;
                connect constant.out1 -> switchBlock.ifIn;
                connect dEMO_FAS_BrakeBooster_Threshold.out1 -> relOp.in2;
                connect brakeForce_pedal_pcIn1 -> relOp.in1;
                connect brakeForce_pedal_pcIn1 -> switchBlock.elseIn;
              }
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              component Brake_Booster brake_Booster;
              component VERSION_INFO vERSION_INFO;
              connect brake_Booster.brakeForceBoosted_pcOut1 -> brakeForceBoosted_pcOut1;
              connect brakeForce_pedal_pcIn1 -> brake_Booster.brakeForce_pedal_pcIn1;
            }
            <<Type="SubSystem">> component Distancewarner {
              ports
                in Double v_Vehicle_msIn1,
                in Double distance_Object_mIn2,
                out Boolean opti_warn_bOut1,
                out Boolean acusti_warn_bOut2;
              <<Type="SubSystem">> component Distancewarner_Function {
                ports
                  in Double v_Vehicle_msIn1,
                  in Double distance_Object_mIn2,
                  out Boolean opti_warn_bOut1,
                  out Boolean acusti_warn_bOut2;
                <<Type="Constant",Value="1">> component DEMO_FAS_Accusti_Threshold_Time {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="1.5">> component DEMO_FAS_Opti_Threshold_Time {
                  ports
                    out Double out1;
                }
                <<Type="Product",Inputs="**">> component Mul {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double opti_warn_bOut1;
                  effect in1 -> opti_warn_bOut1;
                  effect in2 -> opti_warn_bOut1;
                }
                <<Type="Product",Inputs="**">> component Mul1 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Operator="<=",Type="RelationalOperator">> component RelOp {
                  ports
                    in Double in1,
                    in Double opti_warn_bIn2,
                    out Boolean opti_warn_bOut1;
                  effect in1 -> opti_warn_bOut1;
                  effect opti_warn_bIn2 -> opti_warn_bOut1;
                }
                <<Operator="<=",Type="RelationalOperator">> component RelOp2 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean acusti_warn_bOut1;
                  effect in1 -> acusti_warn_bOut1;
                  effect in2 -> acusti_warn_bOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                component DEMO_FAS_Accusti_Threshold_Time dEMO_FAS_Accusti_Threshold_Time;
                component DEMO_FAS_Opti_Threshold_Time dEMO_FAS_Opti_Threshold_Time;
                component Mul mul;
                component Mul1 mul1;
                component RelOp relOp;
                component RelOp2 relOp2;
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
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              component Distancewarner_Function distancewarner_Function;
              component VERSION_INFO vERSION_INFO;
              connect distance_Object_mIn2 -> distancewarner_Function.distance_Object_mIn2;
              connect v_Vehicle_msIn1 -> distancewarner_Function.v_Vehicle_msIn1;
              connect distancewarner_Function.acusti_warn_bOut2 -> acusti_warn_bOut2;
              connect distancewarner_Function.opti_warn_bOut1 -> opti_warn_bOut1;
            }
            <<Type="SubSystem">> component Distronic {
              ports
                in Boolean cC_active_bIn1,
                in Double v_Vehicle_msIn2,
                in Double distance_Object_mIn3,
                in Double distance_statIn4,
                in Double v_Obj_rel_kmhIn5,
                in Boolean fTS_active_bIn6,
                in Double fTS_Abstand_soll_mIn7,
                out Double brakeForce_Distronic_pcOut1,
                out Double decelerator_pcOut2;
              <<Type="Constant",Value="1">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="SubSystem">> component Distronic_Deactive {
                ports
                  out Double brakeForce_Distronic_pcOut1,
                  out Double decelerator_pcOut2;
                <<Type="Constant",Value="0">> component Constant {
                  ports
                    out Double brakeForce_Distronic_pcOut1;
                }
                <<Type="Constant",Value="100">> component Constant1 {
                  ports
                    out Double decelerator_pcOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                component Constant constant;
                component Constant1 constant1;
                component VERSION_INFO vERSION_INFO;
                connect constant1.decelerator_pcOut1 -> decelerator_pcOut2;
                connect constant.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
              }
              <<Type="SubSystem">> component Distronic_Disabled {
                ports
                  out Double brakeForce_Distronic_pcOut1,
                  out Double decelerator_pcOut2;
                <<Type="Constant",Value="0">> component Constant {
                  ports
                    out Double brakeForce_Distronic_pcOut1;
                }
                <<Type="Constant",Value="100">> component Constant1 {
                  ports
                    out Double decelerator_pcOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                component Constant constant;
                component Constant1 constant1;
                component VERSION_INFO vERSION_INFO;
                connect constant1.decelerator_pcOut1 -> decelerator_pcOut2;
                connect constant.brakeForce_Distronic_pcOut1 -> brakeForce_Distronic_pcOut1;
              }
              <<Type="SubSystem">> component Distronic_Enabled {
                ports
                  in Double v_Vehicle_msIn1,
                  in Double distance_Object_mIn2,
                  in Double distance_statIn3,
                  in Double v_Obj_rel_kmhIn4,
                  out Double brakeForce_Distronic_pcOut1,
                  out Double decelerator_pcOut2;
                <<Type="Constant",Value="100">> component Constant {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="2.5">> component Constant1 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="3">> component DEMO_FAS_Distance_Treshold_Far {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="2.5">> component DEMO_FAS_Distance_Treshold_Med {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="2">> component DEMO_FAS_Distance_Treshold_Near {
                  ports
                    out Double out1;
                }
                <<BreakpointsForDimension1="[0,5, 20,50]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[100,100, 10,0]",BreakpointsSpecification="Explicit values">> component LookUpTable {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<BreakpointsForDimension1="[-1,0,10]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[0,0.15,1]",BreakpointsSpecification="Explicit values">> component LookUpTable1 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<BreakpointsForDimension1="[0,20,50,100]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[0,50,90,100]",BreakpointsSpecification="Explicit values">> component LookUpTable2 {
                  ports
                    in Double in1,
                    out Double decelerator_pcOut1;
                  effect in1 -> decelerator_pcOut1;
                }
                <<BreakpointsForDimension1="[-10,0]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[1,0.9]",BreakpointsSpecification="Explicit values">> component LookUpTable3 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<BreakpointsForDimension1="[0,10]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[50,0]",BreakpointsSpecification="Explicit values">> component LookUpTable4 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<Type="Product",Inputs="**">> component Mul {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Product",Inputs="**">> component Mul1 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double brakeForce_Distronic_pcOut1;
                  effect in1 -> brakeForce_Distronic_pcOut1;
                  effect in2 -> brakeForce_Distronic_pcOut1;
                }
                <<Type="Product",Inputs="**">> component Mul2 {
                  ports
                    in Double decelerator_pcIn1,
                    in Double in2,
                    out Double decelerator_pcOut1;
                  effect decelerator_pcIn1 -> decelerator_pcOut1;
                  effect in2 -> decelerator_pcOut1;
                }
                <<Type="MultiPortSwitch",DataPortOrder="One-based contiguous">> component MultiportSwitch {
                  ports
                    in Double in1,
                    in Double in2,
                    in Double in3,
                    in Double in4,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                  effect in3 -> out1;
                  effect in4 -> out1;
                }
                <<Operator="<",Type="RelationalOperator">> component RelOp {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Sum",ListOfSigns="+-+">> component Sum {
                  ports
                    in Double in1,
                    in Double in2,
                    in Double in3,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                  effect in3 -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double brakeForce_Distronic_pcOut1;
                  effect ifIn -> brakeForce_Distronic_pcOut1;
                  effect condition -> brakeForce_Distronic_pcOut1;
                  effect elseIn -> brakeForce_Distronic_pcOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                component Constant constant;
                component Constant1 constant1;
                component DEMO_FAS_Distance_Treshold_Far dEMO_FAS_Distance_Treshold_Far;
                component DEMO_FAS_Distance_Treshold_Med dEMO_FAS_Distance_Treshold_Med;
                component DEMO_FAS_Distance_Treshold_Near dEMO_FAS_Distance_Treshold_Near;
                component LookUpTable lookUpTable;
                component LookUpTable1 lookUpTable1;
                component LookUpTable2 lookUpTable2;
                component LookUpTable3 lookUpTable3;
                component LookUpTable4 lookUpTable4;
                component Mul mul;
                component Mul1 mul1;
                component Mul2 mul2;
                component MultiportSwitch multiportSwitch;
                component RelOp relOp;
                component Sum sum;
                component SwitchBlock switchBlock;
                component VERSION_INFO vERSION_INFO;
                component Condition condition;
                connect condition.out1 -> switchBlock.condition;
                connect relOp.out1 -> condition.in1;
                connect lookUpTable4.out1 -> sum.in3;
                connect lookUpTable3.out1 -> mul2.in2;
                connect lookUpTable2.decelerator_pcOut1 -> mul2.decelerator_pcIn1;
                connect mul2.decelerator_pcOut1 -> decelerator_pcOut2;
                connect constant1.out1 -> relOp.in2;
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
              <<Type="SubSystem">> component Distronic_FTS_Enabled {
                ports
                  in Double distance_Object_mIn1,
                  in Double fTS_Abstand_soll_mIn2,
                  in Double v_Obj_rel_kmhIn3,
                  out Double brakeForce_Distronic_pcOut1,
                  out Double decelerator_pcOut2;
                <<Type="Constant",Value="100">> component Constant {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="2.5">> component Constant1 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="100">> component Constant2 {
                  ports
                    out Double decelerator_pcOut1;
                }
                <<BreakpointsForDimension1="[0,2, 10,20]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[100,100, 10,0]",BreakpointsSpecification="Explicit values">> component LookUpTable {
                  ports
                    in Double in1,
                    out Double brakeForce_Distronic_pcOut1;
                  effect in1 -> brakeForce_Distronic_pcOut1;
                }
                <<BreakpointsForDimension1="[-1,0,10]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[0,0.15,1]",BreakpointsSpecification="Explicit values">> component LookUpTable1 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<Type="Product",Inputs="**">> component Mul1 {
                  ports
                    in Double brakeForce_Distronic_pcIn1,
                    in Double in2,
                    out Double brakeForce_Distronic_pcOut1;
                  effect brakeForce_Distronic_pcIn1 -> brakeForce_Distronic_pcOut1;
                  effect in2 -> brakeForce_Distronic_pcOut1;
                }
                <<Operator="<",Type="RelationalOperator">> component RelOp {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Sum",ListOfSigns="+-">> component Sum {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double brakeForce_Distronic_pcOut1;
                  effect ifIn -> brakeForce_Distronic_pcOut1;
                  effect condition -> brakeForce_Distronic_pcOut1;
                  effect elseIn -> brakeForce_Distronic_pcOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                component Constant constant;
                component Constant1 constant1;
                component Constant2 constant2;
                component LookUpTable lookUpTable;
                component LookUpTable1 lookUpTable1;
                component Mul1 mul1;
                component RelOp relOp;
                component Sum sum;
                component SwitchBlock switchBlock;
                component VERSION_INFO vERSION_INFO;
                component Condition condition;
                connect condition.out1 -> switchBlock.condition;
                connect relOp.out1 -> condition.in1;
                connect constant2.decelerator_pcOut1 -> decelerator_pcOut2;
                connect constant1.out1 -> relOp.in2;
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
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Type="Switch">> component SwitchBlock {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & u2 & ~u3",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock1 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & u2 & u3 ",Type="Condition">> component Condition1 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock2 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & ~u2",Type="Condition">> component Condition2 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock3 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="!(u1 & u2 & ~u3)&&!(u1 & u2 & u3 )&&!(u1 & ~u2)",Type="Condition">> component Condition3 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                ports
                  in Double valueIn,
                  out Double valueOut;
              }
              <<Type="Switch">> component SwitchBlock4 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & u2 & ~u3",Type="Condition">> component Condition4 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock5 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & u2 & u3 ",Type="Condition">> component Condition5 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock6 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1 & ~u2",Type="Condition">> component Condition6 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock7 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="!(u1 & u2 & ~u3)&&!(u1 & u2 & u3 )&&!(u1 & ~u2)",Type="Condition">> component Condition7 {
                ports
                  in Double in1,
                  in Boolean in2,
                  in Boolean in3,
                  out Boolean out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
                ports
                  in Double valueIn,
                  out Double valueOut;
              }
              component Constant constant;
              component Distronic_Deactive distronic_Deactive;
              component Distronic_Disabled distronic_Disabled;
              component Distronic_Enabled distronic_Enabled;
              component Distronic_FTS_Enabled distronic_FTS_Enabled;
              component VERSION_INFO vERSION_INFO;
              component SwitchBlock switchBlock;
              component Condition condition;
              component SwitchBlock1 switchBlock1;
              component Condition1 condition1;
              component SwitchBlock2 switchBlock2;
              component Condition2 condition2;
              component SwitchBlock3 switchBlock3;
              component Condition3 condition3;
              component UnitDelay unitDelay;
              component SwitchBlock4 switchBlock4;
              component Condition4 condition4;
              component SwitchBlock5 switchBlock5;
              component Condition5 condition5;
              component SwitchBlock6 switchBlock6;
              component Condition6 condition6;
              component SwitchBlock7 switchBlock7;
              component Condition7 condition7;
              component UnitDelay1 unitDelay1;
              connect switchBlock.out1 -> brakeForce_Distronic_pcOut1;
              connect switchBlock4.out1 -> decelerator_pcOut2;
              connect constant.out1 -> condition.in1;
              connect cC_active_bIn1 -> condition.in2;
              connect fTS_active_bIn6 -> condition.in3;
              connect condition.out1 -> switchBlock.condition;
              connect distronic_Enabled.brakeForce_Distronic_pcOut1 -> switchBlock.ifIn;
              connect constant.out1 -> condition1.in1;
              connect cC_active_bIn1 -> condition1.in2;
              connect fTS_active_bIn6 -> condition1.in3;
              connect condition1.out1 -> switchBlock1.condition;
              connect distronic_FTS_Enabled.brakeForce_Distronic_pcOut1 -> switchBlock1.ifIn;
              connect switchBlock1.out1 -> switchBlock.elseIn;
              connect constant.out1 -> condition2.in1;
              connect cC_active_bIn1 -> condition2.in2;
              connect fTS_active_bIn6 -> condition2.in3;
              connect condition2.out1 -> switchBlock2.condition;
              connect distronic_Disabled.brakeForce_Distronic_pcOut1 -> switchBlock2.ifIn;
              connect switchBlock2.out1 -> switchBlock1.elseIn;
              connect constant.out1 -> condition3.in1;
              connect cC_active_bIn1 -> condition3.in2;
              connect fTS_active_bIn6 -> condition3.in3;
              connect condition3.out1 -> switchBlock3.condition;
              connect distronic_Deactive.brakeForce_Distronic_pcOut1 -> switchBlock3.ifIn;
              connect switchBlock3.out1 -> switchBlock2.elseIn;
              connect switchBlock.out1 -> unitDelay.valueIn;
              connect unitDelay.valueOut -> switchBlock3.elseIn;
              connect constant.out1 -> condition4.in1;
              connect cC_active_bIn1 -> condition4.in2;
              connect fTS_active_bIn6 -> condition4.in3;
              connect condition4.out1 -> switchBlock4.condition;
              connect distronic_Enabled.decelerator_pcOut2 -> switchBlock4.ifIn;
              connect constant.out1 -> condition5.in1;
              connect cC_active_bIn1 -> condition5.in2;
              connect fTS_active_bIn6 -> condition5.in3;
              connect condition5.out1 -> switchBlock5.condition;
              connect distronic_FTS_Enabled.decelerator_pcOut2 -> switchBlock5.ifIn;
              connect switchBlock5.out1 -> switchBlock4.elseIn;
              connect constant.out1 -> condition6.in1;
              connect cC_active_bIn1 -> condition6.in2;
              connect fTS_active_bIn6 -> condition6.in3;
              connect condition6.out1 -> switchBlock6.condition;
              connect distronic_Disabled.decelerator_pcOut2 -> switchBlock6.ifIn;
              connect switchBlock6.out1 -> switchBlock5.elseIn;
              connect constant.out1 -> condition7.in1;
              connect cC_active_bIn1 -> condition7.in2;
              connect fTS_active_bIn6 -> condition7.in3;
              connect condition7.out1 -> switchBlock7.condition;
              connect distronic_Deactive.decelerator_pcOut2 -> switchBlock7.ifIn;
              connect switchBlock7.out1 -> switchBlock6.elseIn;
              connect switchBlock4.out1 -> unitDelay1.valueIn;
              connect unitDelay1.valueOut -> switchBlock7.elseIn;
              connect fTS_Abstand_soll_mIn7 -> distronic_FTS_Enabled.fTS_Abstand_soll_mIn2;
              connect distance_statIn4 -> distronic_Enabled.distance_statIn3;
              connect v_Vehicle_msIn2 -> distronic_Enabled.v_Vehicle_msIn1;
              connect v_Obj_rel_kmhIn5 -> distronic_Enabled.v_Obj_rel_kmhIn4;
              connect v_Obj_rel_kmhIn5 -> distronic_FTS_Enabled.v_Obj_rel_kmhIn3;
              connect distance_Object_mIn3 -> distronic_Enabled.distance_Object_mIn2;
              connect distance_Object_mIn3 -> distronic_FTS_Enabled.distance_Object_mIn1;
            }
            <<Type="SubSystem">> component EmergencyBrake {
              ports
                in Double v_Vehicle_msIn1,
                in Double distance_Object_mIn2,
                in Double v_Obj_rel_msIn3,
                out Double brakeForce_Emergency_pcOut1,
                out Boolean acusti_warn_bOut2;
              <<Type="Constant",Value="1">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="SubSystem">> component EmergencyBrake_Function {
                ports
                  in Double v_Vehicle_msIn1,
                  in Double distance_Object_mIn2,
                  in Double v_Obj_rel_msIn3,
                  out Double brakeForce_Emergency_pcOut1,
                  out Boolean acusti_warn_bOut2;
                <<Type="Constant",Value="0">> component Constant1 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="0">> component Constant2 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="40">> component DEMO_FAS_Emerg_Brake_Force1 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="80">> component DEMO_FAS_Emerg_Brake_Force2 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="1">> component DEMO_FAS_Emerg_Brake_Time1 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="1.5">> component DEMO_FAS_Emerg_Brake_Time12 {
                  ports
                    out Double out1;
                }
                <<Type="Constant",Value="8">> component DEMO_FAS_Neg_Max_ms {
                  ports
                    out Double out1;
                }
                <<Type="Product",Inputs="*/">> component Div {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Product",Inputs="*/">> component Div1 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Function="max",Type="MinMax">> component MinMax {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double brakeForce_Emergency_pcOut1;
                  effect in1 -> brakeForce_Emergency_pcOut1;
                  effect in2 -> brakeForce_Emergency_pcOut1;
                }
                <<Operator="<",Type="RelationalOperator">> component RelOp {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean acusti_warn_bOut1;
                  effect in1 -> acusti_warn_bOut1;
                  effect in2 -> acusti_warn_bOut1;
                }
                <<Operator="<",Type="RelationalOperator">> component RelOp1 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Boolean out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Saturate",LowerLimit="1",UpperLimit="inf">> component Saturation1 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<Type="Saturate",LowerLimit="1",UpperLimit="inf">> component Saturation2 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<Type="Saturate",LowerLimit="1",UpperLimit="inf">> component Saturation3 {
                  ports
                    in Double in1,
                    out Double out1;
                  effect in1 -> out1;
                }
                <<Type="Sum",ListOfSigns="-+">> component Sum1 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Sum",ListOfSigns="-+">> component Sum2 {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                  effect ifIn -> out1;
                  effect condition -> out1;
                  effect elseIn -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock1 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                  effect ifIn -> out1;
                  effect condition -> out1;
                  effect elseIn -> out1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                component Constant1 constant1;
                component Constant2 constant2;
                component DEMO_FAS_Emerg_Brake_Force1 dEMO_FAS_Emerg_Brake_Force1;
                component DEMO_FAS_Emerg_Brake_Force2 dEMO_FAS_Emerg_Brake_Force2;
                component DEMO_FAS_Emerg_Brake_Time1 dEMO_FAS_Emerg_Brake_Time1;
                component DEMO_FAS_Emerg_Brake_Time12 dEMO_FAS_Emerg_Brake_Time12;
                component DEMO_FAS_Neg_Max_ms dEMO_FAS_Neg_Max_ms;
                component Div div;
                component Div1 div1;
                component MinMax minMax;
                component RelOp relOp;
                component RelOp1 relOp1;
                component Saturation1 saturation1;
                component Saturation2 saturation2;
                component Saturation3 saturation3;
                component Sum1 sum1;
                component Sum2 sum2;
                component SwitchBlock switchBlock;
                component SwitchBlock1 switchBlock1;
                component VERSION_INFO vERSION_INFO;
                component Condition condition;
                component Condition1 condition1;
                connect condition1.out1 -> switchBlock1.condition;
                connect condition.out1 -> switchBlock.condition;
                connect relOp.acusti_warn_bOut1 -> condition.in1;
                connect relOp1.out1 -> condition1.in1;
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
                connect dEMO_FAS_Emerg_Brake_Force2.out1 -> switchBlock1.ifIn;
                connect dEMO_FAS_Emerg_Brake_Force1.out1 -> switchBlock.ifIn;
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
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Condition="u1>0",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                ports
                  in Double valueIn,
                  out Double valueOut;
              }
              <<Condition="u1>0",Type="Condition">> component Condition1 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock1 {
                ports
                  in Boolean ifIn,
                  in Boolean condition,
                  in Boolean elseIn,
                  out Boolean out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
                ports
                  in Boolean valueIn,
                  out Boolean valueOut;
              }
              component Constant constant;
              component EmergencyBrake_Function emergencyBrake_Function;
              component VERSION_INFO vERSION_INFO;
              component Condition condition;
              component SwitchBlock switchBlock;
              component UnitDelay unitDelay;
              component Condition1 condition1;
              component SwitchBlock1 switchBlock1;
              component UnitDelay1 unitDelay1;
              connect constant.out1 -> condition.in1;
              connect condition.out1 -> switchBlock.condition;
              connect emergencyBrake_Function.brakeForce_Emergency_pcOut1 -> switchBlock.ifIn;
              connect switchBlock.out1 -> unitDelay.valueIn;
              connect unitDelay.valueOut -> switchBlock.elseIn;
              connect constant.out1 -> condition1.in1;
              connect condition1.out1 -> switchBlock1.condition;
              connect emergencyBrake_Function.acusti_warn_bOut2 -> switchBlock1.ifIn;
              connect switchBlock1.out1 -> unitDelay1.valueIn;
              connect unitDelay1.valueOut -> switchBlock1.elseIn;
              connect switchBlock1.out1 -> acusti_warn_bOut2;
              connect switchBlock.out1 -> brakeForce_Emergency_pcOut1;
              connect distance_Object_mIn2 -> emergencyBrake_Function.distance_Object_mIn2;
              connect v_Vehicle_msIn1 -> emergencyBrake_Function.v_Vehicle_msIn1;
              connect v_Obj_rel_msIn3 -> emergencyBrake_Function.v_Obj_rel_msIn3;
            }
            <<Type="SubSystem">> component FAS_Input {
              ports
                out Double v_Vehicle_ms,
                out Double distance_Object_m,
                out Double v_Vehicle_ms1,
                out Double distance_Object_m1,
                out Double distance_stat,
                out Double v_Obj_rel_kmh,
                out Double v_Vehicle_ms2,
                out Double distance_Object_m2,
                out Double v_Obj_rel_ms,
                in Boolean parkingBrake_b,
                in Double brakeForce_pedal_pc,
                in Double acceleration_pedal_pc,
                in Boolean cruiseControl_b,
                in Boolean limiter_b,
                in Double leverUp_stat,
                in Double leverDown_stat,
                in Double v_Vehicle_kmh,
                in Double v_Vehicle_kmh1,
                in Double v_Sign_kmh,
                in Boolean sign_b,
                in Double distance_stat1,
                in Double distance_stat2,
                in Double v_Obj_rel_kmh1,
                in Double v_Obj_rel_kmh2,
                in Double v_Obj_rel_kmh3,
                in Double distance_Object_m3,
                in Double distance_Object_m4,
                in Double distance_Object_m5,
                in Double distance_Object_m6,
                out Double v_Vehicle_kmh2,
                out Double v_Obj_rel_kmh4,
                out Double acceleration_pedal_pc1,
                out Double v_Vehicle_kmh3,
                out Double brakeForce_pedal_pc1,
                out Double brakeForce_pedal_pc2,
                out Boolean limiter_b1,
                out Double acceleration_pedal_pc2,
                out Double v_Vehicle_kmh4,
                out Double leverUp_stat1,
                out Double leverDown_stat1,
                out Double v_Sign_kmh1,
                out Boolean sign_b1,
                out Boolean parkingBrake_b1,
                out Double brakeForce_pedal_pc3,
                out Boolean cruiseControl_b1,
                out Double v_Vehicle_kmh5,
                out Boolean limiter_b2,
                out Double leverUp_stat2,
                out Double leverDown_stat2;
              <<Type="Constant",Value="3.6">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="Constant",Value="3.6">> component Constant1 {
                ports
                  out Double out1;
              }
              <<Type="Product",Inputs="*/">> component Div {
                ports
                  in Double v_Vehicle_kmhIn1,
                  in Double in2,
                  out Double v_Vehicle_msOut1;
                effect v_Vehicle_kmhIn1 -> v_Vehicle_msOut1;
                effect in2 -> v_Vehicle_msOut1;
              }
              <<Type="Product",Inputs="*/">> component Div1 {
                ports
                  in Double v_Obj_rel_kmhIn1,
                  in Double in2,
                  out Double v_Obj_rel_msOut1;
                effect v_Obj_rel_kmhIn1 -> v_Obj_rel_msOut1;
                effect in2 -> v_Obj_rel_msOut1;
              }
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              component Constant constant;
              component Constant1 constant1;
              component Div div;
              component Div1 div1;
              component VERSION_INFO vERSION_INFO;
              connect v_Obj_rel_kmh3 -> div1.v_Obj_rel_kmhIn1;
              connect v_Vehicle_kmh1 -> div.v_Vehicle_kmhIn1;
              connect div.v_Vehicle_msOut1 -> v_Vehicle_ms;
              connect distance_Object_m5 -> distance_Object_m;
              connect div.v_Vehicle_msOut1 -> v_Vehicle_ms1;
              connect distance_Object_m4 -> distance_Object_m1;
              connect distance_stat1 -> distance_stat;
              connect v_Obj_rel_kmh2 -> v_Obj_rel_kmh;
              connect div.v_Vehicle_msOut1 -> v_Vehicle_ms2;
              connect distance_Object_m6 -> distance_Object_m2;
              connect div1.v_Obj_rel_msOut1 -> v_Obj_rel_ms;
              connect v_Vehicle_kmh -> v_Vehicle_kmh2;
              connect v_Obj_rel_kmh1 -> v_Obj_rel_kmh4;
              connect acceleration_pedal_pc -> acceleration_pedal_pc1;
              connect v_Vehicle_kmh -> v_Vehicle_kmh3;
              connect brakeForce_pedal_pc -> brakeForce_pedal_pc1;
              connect brakeForce_pedal_pc -> brakeForce_pedal_pc2;
              connect limiter_b -> limiter_b1;
              connect acceleration_pedal_pc -> acceleration_pedal_pc2;
              connect v_Vehicle_kmh -> v_Vehicle_kmh4;
              connect leverUp_stat -> leverUp_stat1;
              connect leverDown_stat -> leverDown_stat1;
              connect v_Sign_kmh -> v_Sign_kmh1;
              connect sign_b -> sign_b1;
              connect parkingBrake_b -> parkingBrake_b1;
              connect brakeForce_pedal_pc -> brakeForce_pedal_pc3;
              connect cruiseControl_b -> cruiseControl_b1;
              connect v_Vehicle_kmh -> v_Vehicle_kmh5;
              connect limiter_b -> limiter_b2;
              connect leverUp_stat -> leverUp_stat2;
              connect leverDown_stat -> leverDown_stat2;
              connect constant1.out1 -> div1.in2;
              connect constant.out1 -> div.in2;
            }
            <<Type="SubSystem">> component FollowToStop {
              ports
                in Double v_Vehicle_kmhIn1,
                in Double v_Obj_rel_kmhIn2,
                out Boolean fTS_active_bOut1,
                out Double fTS_Abstand_soll_mOut2;
              <<Type="Constant",Value="1">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="SubSystem">> component FollowToStop_Function {
                ports
                  in Double v_Vehicle_kmhIn1,
                  in Double v_Obj_rel_kmhIn2,
                  out Boolean fTS_active_bOut1,
                  out Double fTS_Abstand_soll_mOut2;
                <<Type="SubSystem">> component Chart {
                  ports
                    in Double v_Obj_kmhIn1,
                    out Boolean fTS_active_bOut1,
                    out Double abstand_soll_mOut2;
                  <<Type="Demux">> component Demux {
                    ports
                      in Double in1,
                      out Double out1;
                    effect in1 -> out1;
                  }
                  <<Type="S-Function">> component SFunction {
                    ports
                      in Double in1,
                      out Double out1,
                      out Boolean fTS_active_bOut2,
                      out Double abstand_soll_mOut3;
                    effect in1 -> out1;
                    effect in1 -> fTS_active_bOut2;
                    effect in1 -> abstand_soll_mOut3;
                  }
                  <<Type="Terminator">> component Terminator {
                    ports
                      in Double in1;
                  }
                  component Demux demux;
                  component SFunction sFunction;
                  component Terminator terminator;
                  connect sFunction.out1 -> demux.in1;
                  connect demux.out1 -> terminator.in1;
                  connect sFunction.abstand_soll_mOut3 -> abstand_soll_mOut2;
                  connect sFunction.fTS_active_bOut2 -> fTS_active_bOut1;
                  connect v_Obj_kmhIn1 -> sFunction.in1;
                }
                <<Type="Sum",ListOfSigns="+-">> component Sum {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double v_Obj_kmhOut1;
                  effect in1 -> v_Obj_kmhOut1;
                  effect in2 -> v_Obj_kmhOut1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                component Chart chart;
                component Sum sum;
                component VERSION_INFO vERSION_INFO;
                connect chart.abstand_soll_mOut2 -> fTS_Abstand_soll_mOut2;
                connect chart.fTS_active_bOut1 -> fTS_active_bOut1;
                connect sum.v_Obj_kmhOut1 -> chart.v_Obj_kmhIn1;
                connect v_Vehicle_kmhIn1 -> sum.in1;
                connect v_Obj_rel_kmhIn2 -> sum.in2;
              }
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Condition="u1>0",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock {
                ports
                  in Boolean ifIn,
                  in Boolean condition,
                  in Boolean elseIn,
                  out Boolean out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                ports
                  in Boolean valueIn,
                  out Boolean valueOut;
              }
              <<Condition="u1>0",Type="Condition">> component Condition1 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock1 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
                ports
                  in Double valueIn,
                  out Double valueOut;
              }
              component Constant constant;
              component FollowToStop_Function followToStop_Function;
              component VERSION_INFO vERSION_INFO;
              component Condition condition;
              component SwitchBlock switchBlock;
              component UnitDelay unitDelay;
              component Condition1 condition1;
              component SwitchBlock1 switchBlock1;
              component UnitDelay1 unitDelay1;
              connect constant.out1 -> condition.in1;
              connect condition.out1 -> switchBlock.condition;
              connect followToStop_Function.fTS_active_bOut1 -> switchBlock.ifIn;
              connect switchBlock.out1 -> unitDelay.valueIn;
              connect unitDelay.valueOut -> switchBlock.elseIn;
              connect constant.out1 -> condition1.in1;
              connect condition1.out1 -> switchBlock1.condition;
              connect followToStop_Function.fTS_Abstand_soll_mOut2 -> switchBlock1.ifIn;
              connect switchBlock1.out1 -> unitDelay1.valueIn;
              connect unitDelay1.valueOut -> switchBlock1.elseIn;
              connect switchBlock1.out1 -> fTS_Abstand_soll_mOut2;
              connect switchBlock.out1 -> fTS_active_bOut1;
              connect v_Obj_rel_kmhIn2 -> followToStop_Function.v_Obj_rel_kmhIn2;
              connect v_Vehicle_kmhIn1 -> followToStop_Function.v_Vehicle_kmhIn1;
            }
            <<Type="SubSystem">> component Limiter {
              ports
                in Boolean limiter_bIn1,
                in Double acceleration_pedal_pcIn2,
                in Double v_Vehicle_kmhIn3,
                in Double leverUp_statIn4,
                in Double leverDown_statIn5,
                in Double v_Sign_kmhIn6,
                in Boolean sign_bIn7,
                out Double vMax_kmhOut1,
                out Boolean limiter_active_bOut2,
                in Double in1,
                in Double in2,
                out Double out1,
                in Double in3,
                in Double in4,
                in Double in5,
                in Double in6,
                in Double in7,
                in Double in8,
                in Double in9,
                out Double out2;
              <<Type="Constant",Value="1">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="SubSystem">> component Limiter_Function {
                ports
                  in Boolean limiter_bIn1,
                  in Double acceleration_pedal_pcIn2,
                  in Double v_Vehicle_kmhIn3,
                  in Double leverUp_statIn4,
                  in Double leverDown_statIn5,
                  in Double v_Sign_kmhIn6,
                  in Boolean sign_bIn7,
                  out Double vMax_kmhOut1,
                  out Boolean limiter_active_bOut2,
                  in Double in1,
                  out Double out1,
                  in Double in2,
                  in Double in3,
                  in Double in4,
                  in Double in5,
                  in Double in6,
                  in Double in7,
                  out Double out2;
                <<Type="Constant",Value="1">> component Constant {
                  ports
                    out Boolean out1;
                }
                <<Type="SubSystem">> component Limiter_Enabled {
                  ports
                    in Boolean limiter_bIn1,
                    in Double acceleration_pedal_pcIn2,
                    out Double vMax_kmhOut1,
                    out Boolean limiter_active_bOut2,
                    in Double in1;
                  <<Type="SubSystem">> component EdgeRising {
                    ports
                      in Boolean uIn1,
                      in Boolean rIn2,
                      in Boolean iVIn3,
                      out Boolean yOut1;
                    <<Operator="AND",Type="Logic">> component LogOp_A {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_U {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_A logOp_A;
                    component LogOp_N logOp_N;
                    component Memory_U memory_U;
                    component Switch_R switch_R;
                    component Condition condition;
                    connect condition.out1 -> switch_R.condition;
                    connect rIn2 -> condition.in1;
                    connect logOp_N.out1 -> logOp_A.in2;
                    connect logOp_A.out1 -> yOut1;
                    connect uIn1 -> logOp_A.in1;
                    connect uIn1 -> memory_U.in1;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect memory_U.out1 -> switch_R.elseIn;
                    connect iVIn3 -> switch_R.ifIn;
                  }
                  <<Type="SubSystem">> component FalseBlock {
                    ports
                      out Boolean yOut1;
                    <<Type="Constant",Value="0">> component Zero {
                      ports
                        out Boolean out1;
                    }
                    component Zero zero;
                    connect zero.out1 -> yOut1;
                  }
                  <<Type="SubSystem">> component Limiter_Active {
                    ports
                      out Double vMax_kmhOut1,
                      out Boolean limiter_active_bOut2,
                      in Double in1;
                    <<Type="Gain",Gain="1">> component Gain {
                      ports
                        in Double vMax_kmhIn1,
                        out Double vMax_kmhOut1;
                      effect vMax_kmhIn1 -> vMax_kmhOut1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean limiter_enabled_bIn1,
                        in Boolean limiter_enabled_bIn2,
                        out Boolean limiter_active_bOut1;
                      effect limiter_enabled_bIn1 -> limiter_active_bOut1;
                      effect limiter_enabled_bIn2 -> limiter_active_bOut1;
                    }
                    <<Type="SubSystem">> component TrueBlock {
                      ports
                        out Boolean yOut1;
                      <<Type="Constant",Value="1">> component One {
                        ports
                          out Boolean out1;
                      }
                      component One one;
                      connect one.out1 -> yOut1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Gain gain;
                    component LogicalOperator logicalOperator;
                    component TrueBlock trueBlock;
                    component VERSION_INFO vERSION_INFO;
                    connect in1 -> gain.vMax_kmhIn1;
                    connect gain.vMax_kmhOut1 -> vMax_kmhOut1;
                    connect logicalOperator.limiter_active_bOut1 -> limiter_active_bOut2;
                    connect trueBlock.yOut1 -> logicalOperator.limiter_enabled_bIn1;
                    connect trueBlock.yOut1 -> logicalOperator.limiter_enabled_bIn2;
                  }
                  <<Type="SubSystem">> component Limiter_Deactive {
                    ports
                      out Double vMax_kmhOut1,
                      out Boolean limiter_active_bOut2;
                    <<Type="Constant",Value="300">> component Constant {
                      ports
                        out Double vMax_kmhOut1;
                    }
                    <<Type="SubSystem">> component FalseBlock {
                      ports
                        out Boolean yOut1;
                      <<Type="Constant",Value="0">> component Zero {
                        ports
                          out Boolean out1;
                      }
                      component Zero zero;
                      connect zero.out1 -> yOut1;
                    }
                    <<Type="Gain",Gain="1">> component Gain {
                      ports
                        in Double vMax_kmhIn1,
                        out Double vMax_kmhOut1;
                      effect vMax_kmhIn1 -> vMax_kmhOut1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean limiter_active_bIn1,
                        in Boolean limiter_active_bIn2,
                        out Boolean limiter_active_bOut1;
                      effect limiter_active_bIn1 -> limiter_active_bOut1;
                      effect limiter_active_bIn2 -> limiter_active_bOut1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Constant constant;
                    component FalseBlock falseBlock;
                    component Gain gain;
                    component LogicalOperator logicalOperator;
                    component VERSION_INFO vERSION_INFO;
                    connect gain.vMax_kmhOut1 -> vMax_kmhOut1;
                    connect logicalOperator.limiter_active_bOut1 -> limiter_active_bOut2;
                    connect constant.vMax_kmhOut1 -> gain.vMax_kmhIn1;
                    connect falseBlock.yOut1 -> logicalOperator.limiter_active_bIn2;
                    connect falseBlock.yOut1 -> logicalOperator.limiter_active_bIn1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator1 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Type="SubSystem">> component RSFlipFlop {
                    ports
                      in Boolean sIn1,
                      in Boolean rIn2,
                      out Boolean qOut1,
                      out Boolean nOT_QOut2;
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_Q {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Constant",Value="1">> component One_S {
                      ports
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_S {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Constant",Value="0">> component Zero_R {
                      ports
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_N logOp_N;
                    component Memory_Q memory_Q;
                    component One_S one_S;
                    component Switch_R switch_R;
                    component Switch_S switch_S;
                    component Zero_R zero_R;
                    component Condition condition;
                    component Condition1 condition1;
                    connect condition.out1 -> switch_S.condition;
                    connect condition1.out1 -> switch_R.condition;
                    connect sIn1 -> condition.in1;
                    connect rIn2 -> condition1.in1;
                    connect memory_Q.out1 -> switch_S.elseIn;
                    connect switch_S.out1 -> switch_R.elseIn;
                    connect one_S.out1 -> switch_S.ifIn;
                    connect switch_R.out1 -> memory_Q.in1;
                    connect switch_R.out1 -> qOut1;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect zero_R.out1 -> switch_R.ifIn;
                    connect logOp_N.out1 -> nOT_QOut2;
                  }
                  <<Type="Terminator">> component Terminator {
                    ports
                      in Boolean in1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  <<Type="Switch">> component SwitchBlock {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u2 & u1 < 90",Type="Condition">> component Condition {
                    ports
                      in Double in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock1 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="!(u2 & u1 < 90)",Type="Condition">> component Condition1 {
                    ports
                      in Double in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                    ports
                      in Double valueIn,
                      out Double valueOut;
                  }
                  <<Type="Switch">> component SwitchBlock2 {
                    ports
                      in Boolean ifIn,
                      in Boolean condition,
                      in Boolean elseIn,
                      out Boolean out1;
                  }
                  <<Condition="u2 & u1 < 90",Type="Condition">> component Condition2 {
                    ports
                      in Double in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock3 {
                    ports
                      in Boolean ifIn,
                      in Boolean condition,
                      in Boolean elseIn,
                      out Boolean out1;
                  }
                  <<Condition="!(u2 & u1 < 90)",Type="Condition">> component Condition3 {
                    ports
                      in Double in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
                    ports
                      in Boolean valueIn,
                      out Boolean valueOut;
                  }
                  component EdgeRising edgeRising;
                  component FalseBlock falseBlock;
                  component Limiter_Active limiter_Active;
                  component Limiter_Deactive limiter_Deactive;
                  component LogicalOperator1 logicalOperator1;
                  component LogicalOperator2 logicalOperator2;
                  component RSFlipFlop rSFlipFlop;
                  component Terminator terminator;
                  component VERSION_INFO vERSION_INFO;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  component SwitchBlock1 switchBlock1;
                  component Condition1 condition1;
                  component UnitDelay unitDelay;
                  component SwitchBlock2 switchBlock2;
                  component Condition2 condition2;
                  component SwitchBlock3 switchBlock3;
                  component Condition3 condition3;
                  component UnitDelay1 unitDelay1;
                  connect switchBlock.out1 -> vMax_kmhOut1;
                  connect switchBlock2.out1 -> limiter_active_bOut2;
                  connect in1 -> limiter_Active.in1;
                  connect acceleration_pedal_pcIn2 -> condition.in1;
                  connect rSFlipFlop.qOut1 -> condition.in2;
                  connect condition.out1 -> switchBlock.condition;
                  connect limiter_Active.vMax_kmhOut1 -> switchBlock.ifIn;
                  connect acceleration_pedal_pcIn2 -> condition1.in1;
                  connect rSFlipFlop.qOut1 -> condition1.in2;
                  connect condition1.out1 -> switchBlock1.condition;
                  connect limiter_Deactive.vMax_kmhOut1 -> switchBlock1.ifIn;
                  connect switchBlock1.out1 -> switchBlock.elseIn;
                  connect switchBlock.out1 -> unitDelay.valueIn;
                  connect unitDelay.valueOut -> switchBlock1.elseIn;
                  connect acceleration_pedal_pcIn2 -> condition2.in1;
                  connect rSFlipFlop.qOut1 -> condition2.in2;
                  connect condition2.out1 -> switchBlock2.condition;
                  connect limiter_Active.limiter_active_bOut2 -> switchBlock2.ifIn;
                  connect acceleration_pedal_pcIn2 -> condition3.in1;
                  connect rSFlipFlop.qOut1 -> condition3.in2;
                  connect condition3.out1 -> switchBlock3.condition;
                  connect limiter_Deactive.limiter_active_bOut2 -> switchBlock3.ifIn;
                  connect switchBlock3.out1 -> switchBlock2.elseIn;
                  connect switchBlock2.out1 -> unitDelay1.valueIn;
                  connect unitDelay1.valueOut -> switchBlock3.elseIn;
                  connect logicalOperator1.out1 -> edgeRising.rIn2;
                  connect falseBlock.yOut1 -> edgeRising.iVIn3;
                  connect edgeRising.yOut1 -> rSFlipFlop.sIn1;
                  connect limiter_bIn1 -> edgeRising.uIn1;
                  connect limiter_bIn1 -> logicalOperator1.in1;
                  connect limiter_bIn1 -> logicalOperator2.in1;
                  connect rSFlipFlop.nOT_QOut2 -> terminator.in1;
                  connect logicalOperator2.out1 -> rSFlipFlop.rIn2;
                }
                <<Type="SubSystem">> component Limiter_InitialSetValue {
                  ports
                    in Double v_Vehicle_kmhIn1,
                    out Double out1,
                    out Double out2,
                    out Double out3,
                    out Double out4,
                    out Double out5,
                    out Double out6;
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  component VERSION_INFO vERSION_INFO;
                  connect v_Vehicle_kmhIn1 -> out1;
                  connect v_Vehicle_kmhIn1 -> out2;
                  connect v_Vehicle_kmhIn1 -> out3;
                  connect v_Vehicle_kmhIn1 -> out4;
                  connect v_Vehicle_kmhIn1 -> out5;
                  connect v_Vehicle_kmhIn1 -> out6;
                }
                <<Type="SubSystem">> component Limiter_SetValue {
                  ports
                    in Double leverUp_statIn1,
                    in Double leverDown_statIn2,
                    in Double in1,
                    in Double in2,
                    in Double in3,
                    out Double out1,
                    in Double in4,
                    out Double out2;
                  <<Type="Constant",Value="0">> component Constant1 {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="0">> component Constant3 {
                    ports
                      out Double out1;
                  }
                  <<Operator=">",Type="RelationalOperator">> component RelOp1 {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator=">",Type="RelationalOperator">> component RelOp3 {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  <<Type="SubSystem">> component V_LimSetValueMinus {
                    ports
                      in Double v_LimiterSetValueIn1,
                      out Double out1,
                      out Double out2;
                    <<Type="Constant",Value="5">> component Parameter {
                      ports
                        out Double out1;
                    }
                    <<Type="Sum",ListOfSigns="+-">> component Sum {
                      ports
                        in Double in1,
                        in Double in2,
                        out Double out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Parameter parameter;
                    component Sum sum;
                    component VERSION_INFO vERSION_INFO;
                    connect sum.out1 -> out1;
                    connect sum.out1 -> out2;
                    connect parameter.out1 -> sum.in2;
                    connect v_LimiterSetValueIn1 -> sum.in1;
                  }
                  <<Type="SubSystem">> component V_LimSetValuePlus {
                    ports
                      in Double v_LimiterSetValueIn1,
                      out Double out1,
                      out Double out2;
                    <<Type="Constant",Value="5">> component Parameter {
                      ports
                        out Double out1;
                    }
                    <<Type="Sum",ListOfSigns="++">> component Sum {
                      ports
                        in Double in1,
                        in Double in2,
                        out Double out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Parameter parameter;
                    component Sum sum;
                    component VERSION_INFO vERSION_INFO;
                    connect sum.out1 -> out1;
                    connect sum.out1 -> out2;
                    connect parameter.out1 -> sum.in2;
                    connect v_LimiterSetValueIn1 -> sum.in1;
                  }
                  <<Type="SubSystem">> component RisingEdgeDetector {
                    ports
                      in Boolean in1In1,
                      out Boolean out1Out1;
                    <<Type="SubSystem">> component CompareToZero {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero1 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero2 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="<",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero3 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">=",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero4 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="Constant",Value="1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="Constant",Value="0">> component Constant1 {
                      ports
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        in Boolean in3,
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory1 {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                    }
                    <<Condition="u2 > 0",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CompareToZero compareToZero;
                    component CompareToZero1 compareToZero1;
                    component CompareToZero2 compareToZero2;
                    component CompareToZero3 compareToZero3;
                    component CompareToZero4 compareToZero4;
                    component Constant constant;
                    component Constant1 constant1;
                    component LogicalOperator logicalOperator;
                    component LogicalOperator1 logicalOperator1;
                    component LogicalOperator2 logicalOperator2;
                    component Memory memory;
                    component Memory1 memory1;
                    component Memory2 memory2;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    connect switchBlock.out1 -> out1Out1;
                    connect compareToZero4.yOut1 -> logicalOperator.in3;
                    connect memory2.out1 -> compareToZero4.uIn1;
                    connect logicalOperator.out1 -> logicalOperator2.in1;
                    connect constant1.out1 -> switchBlock.elseIn;
                    connect constant.out1 -> memory1.in1;
                    connect condition.out1 -> switchBlock.condition;
                    connect logicalOperator2.out1 -> switchBlock.ifIn;
                    connect logicalOperator2.out1 -> memory2.in1;
                    connect logicalOperator1.out1 -> logicalOperator2.in2;
                    connect compareToZero3.yOut1 -> logicalOperator1.in2;
                    connect compareToZero2.yOut1 -> logicalOperator1.in1;
                    connect compareToZero1.yOut1 -> logicalOperator.in2;
                    connect compareToZero.yOut1 -> logicalOperator.in1;
                    connect memory.out1 -> compareToZero.uIn1;
                    connect memory.out1 -> compareToZero2.uIn1;
                    connect in1In1 -> memory.in1;
                    connect in1In1 -> compareToZero1.uIn1;
                    connect in1In1 -> compareToZero3.uIn1;
                    connect memory1.out1 -> condition.in1;
                  }
                  <<Type="SubSystem">> component RisingEdgeDetector1 {
                    ports
                      in Boolean in1In1,
                      out Boolean out1Out1;
                    <<Type="SubSystem">> component CompareToZero {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero1 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero2 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="<",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero3 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">=",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero4 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="Constant",Value="1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="Constant",Value="0">> component Constant1 {
                      ports
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        in Boolean in3,
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory1 {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                    }
                    <<Condition="u2 > 0",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CompareToZero compareToZero;
                    component CompareToZero1 compareToZero1;
                    component CompareToZero2 compareToZero2;
                    component CompareToZero3 compareToZero3;
                    component CompareToZero4 compareToZero4;
                    component Constant constant;
                    component Constant1 constant1;
                    component LogicalOperator logicalOperator;
                    component LogicalOperator1 logicalOperator1;
                    component LogicalOperator2 logicalOperator2;
                    component Memory memory;
                    component Memory1 memory1;
                    component Memory2 memory2;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    connect switchBlock.out1 -> out1Out1;
                    connect compareToZero4.yOut1 -> logicalOperator.in3;
                    connect memory2.out1 -> compareToZero4.uIn1;
                    connect logicalOperator.out1 -> logicalOperator2.in1;
                    connect constant1.out1 -> switchBlock.elseIn;
                    connect constant.out1 -> memory1.in1;
                    connect condition.out1 -> switchBlock.condition;
                    connect logicalOperator2.out1 -> switchBlock.ifIn;
                    connect logicalOperator2.out1 -> memory2.in1;
                    connect logicalOperator1.out1 -> logicalOperator2.in2;
                    connect compareToZero3.yOut1 -> logicalOperator1.in2;
                    connect compareToZero2.yOut1 -> logicalOperator1.in1;
                    connect compareToZero1.yOut1 -> logicalOperator.in2;
                    connect compareToZero.yOut1 -> logicalOperator.in1;
                    connect memory.out1 -> compareToZero.uIn1;
                    connect memory.out1 -> compareToZero2.uIn1;
                    connect in1In1 -> memory.in1;
                    connect in1In1 -> compareToZero1.uIn1;
                    connect in1In1 -> compareToZero3.uIn1;
                    connect memory1.out1 -> condition.in1;
                  }
                  <<Type="Switch">> component SwitchBlock {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock1 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition1 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock2 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock3 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition3 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  component Constant1 constant1;
                  component Constant3 constant3;
                  component RelOp1 relOp1;
                  component RelOp3 relOp3;
                  component VERSION_INFO vERSION_INFO;
                  component V_LimSetValueMinus v_LimSetValueMinus;
                  component V_LimSetValuePlus v_LimSetValuePlus;
                  component RisingEdgeDetector risingEdgeDetector;
                  component RisingEdgeDetector1 risingEdgeDetector1;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  component SwitchBlock1 switchBlock1;
                  component Condition1 condition1;
                  component SwitchBlock2 switchBlock2;
                  component Condition2 condition2;
                  component SwitchBlock3 switchBlock3;
                  component Condition3 condition3;
                  connect in2 -> v_LimSetValueMinus.v_LimiterSetValueIn1;
                  connect in1 -> v_LimSetValuePlus.v_LimiterSetValueIn1;
                  connect relOp3.out1 -> risingEdgeDetector.in1In1;
                  connect relOp1.out1 -> risingEdgeDetector1.in1In1;
                  connect risingEdgeDetector.out1Out1 -> condition.in1;
                  connect v_LimSetValuePlus.out1 -> switchBlock.ifIn;
                  connect condition.out1 -> switchBlock.condition;
                  connect risingEdgeDetector1.out1Out1 -> condition1.in1;
                  connect v_LimSetValueMinus.out1 -> switchBlock1.ifIn;
                  connect condition1.out1 -> switchBlock1.condition;
                  connect in3 -> switchBlock1.elseIn;
                  connect switchBlock1.out1 -> switchBlock.elseIn;
                  connect switchBlock.out1 -> out1;
                  connect risingEdgeDetector.out1Out1 -> condition2.in1;
                  connect v_LimSetValuePlus.out2 -> switchBlock2.ifIn;
                  connect condition2.out1 -> switchBlock2.condition;
                  connect risingEdgeDetector1.out1Out1 -> condition3.in1;
                  connect v_LimSetValueMinus.out2 -> switchBlock3.ifIn;
                  connect condition3.out1 -> switchBlock3.condition;
                  connect in4 -> switchBlock3.elseIn;
                  connect switchBlock3.out1 -> switchBlock2.elseIn;
                  connect switchBlock2.out1 -> out2;
                  connect constant1.out1 -> relOp1.in2;
                  connect constant3.out1 -> relOp3.in2;
                  connect leverUp_statIn1 -> relOp3.in1;
                  connect leverDown_statIn2 -> relOp1.in1;
                }
                <<Type="SubSystem">> component Limiter_StartUpSetValue {
                  ports
                    out Double out1;
                  <<Type="Constant",Value="300">> component Constant {
                    ports
                      out Double out1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  component Constant constant;
                  component VERSION_INFO vERSION_INFO;
                  connect constant.out1 -> out1;
                }
                <<Operator="AND",Type="Logic">> component LogicalOperator {
                  ports
                    in Boolean in1,
                    in Boolean in2,
                    out Boolean out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Function="min",Type="MinMax">> component MinMax {
                  ports
                    in Double in1,
                    in Double in2,
                    out Double out1;
                  effect in1 -> out1;
                  effect in2 -> out1;
                }
                <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double vMax_kmhOut1;
                  effect ifIn -> vMax_kmhOut1;
                  effect condition -> vMax_kmhOut1;
                  effect elseIn -> vMax_kmhOut1;
                }
                <<Type="SubSystem">> component SysInit {
                  ports
                    out Double yOut1;
                  <<Type="UnitDelay",InitialCondition="1">> component Memory_Init {
                    ports
                      in Double in1,
                      out Double out1;
                    effect in1 -> out1;
                  }
                  <<Type="Constant",Value="0">> component Zero_Init {
                    ports
                      out Double out1;
                  }
                  component Memory_Init memory_Init;
                  component Zero_Init zero_Init;
                  connect memory_Init.out1 -> yOut1;
                  connect zero_Init.out1 -> memory_Init.in1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                <<Type="SubSystem">> component RisingEdgeDetector {
                  ports
                    in Boolean in1In1,
                    out Boolean out1Out1;
                  <<Type="SubSystem">> component CompareToZero {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator="==",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero1 {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator=">",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero2 {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator="<",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero3 {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator=">=",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero4 {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator="==",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="Constant",Value="1">> component Constant {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="0">> component Constant1 {
                    ports
                      out Boolean out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      in Boolean in3,
                      out Boolean out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="Memory">> component Memory {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Memory">> component Memory1 {
                    ports
                      in Double in1,
                      out Double out1;
                  }
                  <<Type="Memory">> component Memory2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                    ports
                      in Boolean ifIn,
                      in Boolean condition,
                      in Boolean elseIn,
                      out Boolean out1;
                  }
                  <<Condition="u2 > 0",Type="Condition">> component Condition {
                    ports
                      in Double in1,
                      out Boolean out1;
                  }
                  component CompareToZero compareToZero;
                  component CompareToZero1 compareToZero1;
                  component CompareToZero2 compareToZero2;
                  component CompareToZero3 compareToZero3;
                  component CompareToZero4 compareToZero4;
                  component Constant constant;
                  component Constant1 constant1;
                  component LogicalOperator logicalOperator;
                  component LogicalOperator1 logicalOperator1;
                  component LogicalOperator2 logicalOperator2;
                  component Memory memory;
                  component Memory1 memory1;
                  component Memory2 memory2;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  connect switchBlock.out1 -> out1Out1;
                  connect compareToZero4.yOut1 -> logicalOperator.in3;
                  connect memory2.out1 -> compareToZero4.uIn1;
                  connect logicalOperator.out1 -> logicalOperator2.in1;
                  connect constant1.out1 -> switchBlock.elseIn;
                  connect constant.out1 -> memory1.in1;
                  connect condition.out1 -> switchBlock.condition;
                  connect logicalOperator2.out1 -> switchBlock.ifIn;
                  connect logicalOperator2.out1 -> memory2.in1;
                  connect logicalOperator1.out1 -> logicalOperator2.in2;
                  connect compareToZero3.yOut1 -> logicalOperator1.in2;
                  connect compareToZero2.yOut1 -> logicalOperator1.in1;
                  connect compareToZero1.yOut1 -> logicalOperator.in2;
                  connect compareToZero.yOut1 -> logicalOperator.in1;
                  connect memory.out1 -> compareToZero.uIn1;
                  connect memory.out1 -> compareToZero2.uIn1;
                  connect in1In1 -> memory.in1;
                  connect in1In1 -> compareToZero1.uIn1;
                  connect in1In1 -> compareToZero3.uIn1;
                  connect memory1.out1 -> condition.in1;
                }
                <<Type="SubSystem">> component RisingEdgeDetector1 {
                  ports
                    in Double in1In1,
                    out Boolean out1Out1;
                  <<Type="SubSystem">> component CompareToZero {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator="==",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero1 {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator=">",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero2 {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator="<",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero3 {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator=">=",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero4 {
                    ports
                      in Boolean uIn1,
                      out Boolean yOut1;
                    <<Operator="==",Type="RelationalOperator">> component Compare {
                      ports
                        in Boolean in1,
                        in Double in2,
                        out Boolean out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="Constant",Value="1">> component Constant {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="0">> component Constant1 {
                    ports
                      out Boolean out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      in Boolean in3,
                      out Boolean out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                  }
                  <<Type="Memory">> component Memory {
                    ports
                      in Double in1,
                      out Double out1;
                  }
                  <<Type="Memory">> component Memory1 {
                    ports
                      in Double in1,
                      out Double out1;
                  }
                  <<Type="Memory">> component Memory2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                    ports
                      in Boolean ifIn,
                      in Boolean condition,
                      in Boolean elseIn,
                      out Boolean out1;
                  }
                  <<Condition="u2 > 0",Type="Condition">> component Condition {
                    ports
                      in Double in1,
                      out Boolean out1;
                  }
                  component CompareToZero compareToZero;
                  component CompareToZero1 compareToZero1;
                  component CompareToZero2 compareToZero2;
                  component CompareToZero3 compareToZero3;
                  component CompareToZero4 compareToZero4;
                  component Constant constant;
                  component Constant1 constant1;
                  component LogicalOperator logicalOperator;
                  component LogicalOperator1 logicalOperator1;
                  component LogicalOperator2 logicalOperator2;
                  component Memory memory;
                  component Memory1 memory1;
                  component Memory2 memory2;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  connect switchBlock.out1 -> out1Out1;
                  connect compareToZero4.yOut1 -> logicalOperator.in3;
                  connect memory2.out1 -> compareToZero4.uIn1;
                  connect logicalOperator.out1 -> logicalOperator2.in1;
                  connect constant1.out1 -> switchBlock.elseIn;
                  connect constant.out1 -> memory1.in1;
                  connect condition.out1 -> switchBlock.condition;
                  connect logicalOperator2.out1 -> switchBlock.ifIn;
                  connect logicalOperator2.out1 -> memory2.in1;
                  connect logicalOperator1.out1 -> logicalOperator2.in2;
                  connect compareToZero3.yOut1 -> logicalOperator1.in2;
                  connect compareToZero2.yOut1 -> logicalOperator1.in1;
                  connect compareToZero1.yOut1 -> logicalOperator.in2;
                  connect compareToZero.yOut1 -> logicalOperator.in1;
                  connect memory.out1 -> compareToZero.uIn1;
                  connect memory.out1 -> compareToZero2.uIn1;
                  connect in1In1 -> memory.in1;
                  connect in1In1 -> compareToZero1.uIn1;
                  connect in1In1 -> compareToZero3.uIn1;
                  connect memory1.out1 -> condition.in1;
                }
                <<Type="Switch">> component SwitchBlock1 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock2 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition1 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock3 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition2 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock4 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition3 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock5 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition4 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock6 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition5 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock7 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition6 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock8 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition7 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Type="Switch">> component SwitchBlock9 {
                  ports
                    in Double ifIn,
                    in Boolean condition,
                    in Double elseIn,
                    out Double out1;
                }
                <<Condition="u1>0",Type="Condition">> component Condition8 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                <<Condition="u2 >= 1",Type="Condition">> component Condition9 {
                  ports
                    in Boolean in1,
                    out Boolean out1;
                }
                component Constant constant;
                component Limiter_Enabled limiter_Enabled;
                component Limiter_InitialSetValue limiter_InitialSetValue;
                component Limiter_SetValue limiter_SetValue;
                component Limiter_StartUpSetValue limiter_StartUpSetValue;
                component LogicalOperator logicalOperator;
                component MinMax minMax;
                component SwitchBlock switchBlock;
                component SysInit sysInit;
                component VERSION_INFO vERSION_INFO;
                component RisingEdgeDetector risingEdgeDetector;
                component RisingEdgeDetector1 risingEdgeDetector1;
                component SwitchBlock1 switchBlock1;
                component Condition condition;
                component SwitchBlock2 switchBlock2;
                component Condition1 condition1;
                component SwitchBlock3 switchBlock3;
                component Condition2 condition2;
                component SwitchBlock4 switchBlock4;
                component Condition3 condition3;
                component SwitchBlock5 switchBlock5;
                component Condition4 condition4;
                component SwitchBlock6 switchBlock6;
                component Condition5 condition5;
                component SwitchBlock7 switchBlock7;
                component Condition6 condition6;
                component SwitchBlock8 switchBlock8;
                component Condition7 condition7;
                component SwitchBlock9 switchBlock9;
                component Condition8 condition8;
                component Condition9 condition9;
                connect condition9.out1 -> switchBlock.condition;
                connect limiter_bIn1 -> risingEdgeDetector.in1In1;
                connect sysInit.yOut1 -> risingEdgeDetector1.in1In1;
                connect risingEdgeDetector1.out1Out1 -> condition.in1;
                connect limiter_StartUpSetValue.out1 -> switchBlock1.ifIn;
                connect condition.out1 -> switchBlock1.condition;
                connect in1 -> switchBlock1.elseIn;
                connect switchBlock1.out1 -> out1;
                connect risingEdgeDetector.out1Out1 -> condition1.in1;
                connect limiter_InitialSetValue.out1 -> switchBlock2.ifIn;
                connect condition1.out1 -> switchBlock2.condition;
                connect in2 -> switchBlock2.elseIn;
                connect switchBlock2.out1 -> limiter_SetValue.in1;
                connect risingEdgeDetector.out1Out1 -> condition2.in1;
                connect limiter_InitialSetValue.out2 -> switchBlock3.ifIn;
                connect condition2.out1 -> switchBlock3.condition;
                connect in3 -> switchBlock3.elseIn;
                connect switchBlock3.out1 -> limiter_SetValue.in2;
                connect risingEdgeDetector.out1Out1 -> condition3.in1;
                connect limiter_InitialSetValue.out3 -> switchBlock4.ifIn;
                connect condition3.out1 -> switchBlock4.condition;
                connect in4 -> switchBlock4.elseIn;
                connect switchBlock4.out1 -> limiter_SetValue.in3;
                connect limiter_bIn1 -> condition4.in1;
                connect limiter_SetValue.out1 -> switchBlock5.ifIn;
                connect condition4.out1 -> switchBlock5.condition;
                connect risingEdgeDetector.out1Out1 -> condition5.in1;
                connect limiter_InitialSetValue.out4 -> switchBlock6.ifIn;
                connect condition5.out1 -> switchBlock6.condition;
                connect in5 -> switchBlock6.elseIn;
                connect switchBlock6.out1 -> switchBlock5.elseIn;
                connect switchBlock5.out1 -> limiter_Enabled.in1;
                connect risingEdgeDetector.out1Out1 -> condition6.in1;
                connect limiter_InitialSetValue.out5 -> switchBlock7.ifIn;
                connect condition6.out1 -> switchBlock7.condition;
                connect in6 -> switchBlock7.elseIn;
                connect switchBlock7.out1 -> limiter_SetValue.in4;
                connect limiter_bIn1 -> condition7.in1;
                connect limiter_SetValue.out2 -> switchBlock8.ifIn;
                connect condition7.out1 -> switchBlock8.condition;
                connect risingEdgeDetector.out1Out1 -> condition8.in1;
                connect limiter_InitialSetValue.out6 -> switchBlock9.ifIn;
                connect condition8.out1 -> switchBlock9.condition;
                connect in7 -> switchBlock9.elseIn;
                connect switchBlock9.out1 -> switchBlock8.elseIn;
                connect switchBlock8.out1 -> out2;
                connect logicalOperator.out1 -> condition9.in1;
                connect constant.out1 -> logicalOperator.in2;
                connect v_Sign_kmhIn6 -> minMax.in1;
                connect minMax.out1 -> switchBlock.ifIn;
                connect sign_bIn7 -> logicalOperator.in1;
                connect switchBlock.vMax_kmhOut1 -> vMax_kmhOut1;
                connect limiter_Enabled.limiter_active_bOut2 -> limiter_active_bOut2;
                connect acceleration_pedal_pcIn2 -> limiter_Enabled.acceleration_pedal_pcIn2;
                connect limiter_bIn1 -> limiter_Enabled.limiter_bIn1;
                connect limiter_Enabled.vMax_kmhOut1 -> switchBlock.elseIn;
                connect limiter_Enabled.vMax_kmhOut1 -> minMax.in2;
                connect v_Vehicle_kmhIn3 -> limiter_InitialSetValue.v_Vehicle_kmhIn1;
                connect leverUp_statIn4 -> limiter_SetValue.leverUp_statIn1;
                connect leverDown_statIn5 -> limiter_SetValue.leverDown_statIn2;
              }
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Type="Switch">> component SwitchBlock {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock1 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition1 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition2 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock2 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                ports
                  in Double valueIn,
                  out Double valueOut;
              }
              <<Condition="u1>0",Type="Condition">> component Condition3 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock3 {
                ports
                  in Boolean ifIn,
                  in Boolean condition,
                  in Boolean elseIn,
                  out Boolean out1;
              }
              <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
                ports
                  in Boolean valueIn,
                  out Boolean valueOut;
              }
              component Constant constant;
              component Limiter_Function limiter_Function;
              component VERSION_INFO vERSION_INFO;
              component SwitchBlock switchBlock;
              component Condition condition;
              component SwitchBlock1 switchBlock1;
              component Condition1 condition1;
              component Condition2 condition2;
              component SwitchBlock2 switchBlock2;
              component UnitDelay unitDelay;
              component Condition3 condition3;
              component SwitchBlock3 switchBlock3;
              component UnitDelay1 unitDelay1;
              connect in1 -> limiter_Function.in1;
              connect constant.out1 -> condition.in1;
              connect limiter_Function.out1 -> switchBlock.ifIn;
              connect condition.out1 -> switchBlock.condition;
              connect in2 -> switchBlock.elseIn;
              connect switchBlock.out1 -> out1;
              connect in3 -> limiter_Function.in2;
              connect in4 -> limiter_Function.in3;
              connect in5 -> limiter_Function.in4;
              connect in6 -> limiter_Function.in5;
              connect in7 -> limiter_Function.in6;
              connect in8 -> limiter_Function.in7;
              connect constant.out1 -> condition1.in1;
              connect limiter_Function.out2 -> switchBlock1.ifIn;
              connect condition1.out1 -> switchBlock1.condition;
              connect in9 -> switchBlock1.elseIn;
              connect switchBlock1.out1 -> out2;
              connect constant.out1 -> condition2.in1;
              connect condition2.out1 -> switchBlock2.condition;
              connect limiter_Function.vMax_kmhOut1 -> switchBlock2.ifIn;
              connect switchBlock2.out1 -> unitDelay.valueIn;
              connect unitDelay.valueOut -> switchBlock2.elseIn;
              connect constant.out1 -> condition3.in1;
              connect condition3.out1 -> switchBlock3.condition;
              connect limiter_Function.limiter_active_bOut2 -> switchBlock3.ifIn;
              connect switchBlock3.out1 -> unitDelay1.valueIn;
              connect unitDelay1.valueOut -> switchBlock3.elseIn;
              connect switchBlock3.out1 -> limiter_active_bOut2;
              connect switchBlock2.out1 -> vMax_kmhOut1;
              connect sign_bIn7 -> limiter_Function.sign_bIn7;
              connect v_Sign_kmhIn6 -> limiter_Function.v_Sign_kmhIn6;
              connect leverDown_statIn5 -> limiter_Function.leverDown_statIn5;
              connect leverUp_statIn4 -> limiter_Function.leverUp_statIn4;
              connect v_Vehicle_kmhIn3 -> limiter_Function.v_Vehicle_kmhIn3;
              connect acceleration_pedal_pcIn2 -> limiter_Function.acceleration_pedal_pcIn2;
              connect limiter_bIn1 -> limiter_Function.limiter_bIn1;
            }
            <<Operator="OR",Type="Logic">> component LogOp {
              ports
                in Boolean acusti_warn_bIn1,
                in Boolean acusti_warn_bIn2,
                out Boolean acusti_warn_bOut1;
              effect acusti_warn_bIn1 -> acusti_warn_bOut1;
              effect acusti_warn_bIn2 -> acusti_warn_bOut1;
            }
            <<Type="SubSystem">> component Tempomat {
              ports
                in Boolean parkingBrake_bIn1,
                in Double brakeForce_pedal_pcIn2,
                in Boolean cruiseControl_bIn3,
                in Double v_Vehicle_kmhIn4,
                in Boolean limiter_bIn5,
                in Double leverUp_StatIn6,
                in Double leverDown_StatIn7,
                in Boolean fTS_active_bIn8,
                out Double v_CC_delta_kmhOut1,
                out Boolean cC_active_bOut2,
                in Double in1,
                in Double in2,
                in Double in3,
                in Double in4,
                in Double in5,
                in Double in6,
                in Double in7,
                in Double in8,
                in Double in9,
                in Double in10,
                in Double in11,
                in Double in12,
                in Double in13,
                in Double in14,
                in Double in15,
                in Double in16,
                in Double in17,
                in Double in18,
                in Double in19,
                in Double in20,
                in Double in21,
                in Double in22,
                in Double in23,
                in Double in24,
                in Double in25,
                in Double in26,
                in Double in27,
                in Double in28,
                in Double in29,
                in Double in30,
                in Double in31,
                in Double in32,
                in Double in33,
                in Double in34,
                in Double in35,
                in Double in36,
                in Double in37,
                in Double in38,
                out Double out1,
                in Double in39,
                in Double in40,
                in Double in41,
                in Double in42,
                in Double in43,
                in Double in44,
                in Double in45,
                in Double in46,
                in Double in47,
                in Double in48,
                in Double in49,
                out Double out2;
              <<Type="Constant",Value="1">> component Constant {
                ports
                  out Double out1;
              }
              <<Type="SubSystem">> component Tempomat_Function {
                ports
                  in Boolean parkingBrake_bIn1,
                  in Double brakeForce_pedal_pcIn2,
                  in Boolean cruiseControl_bIn3,
                  in Double v_Vehicle_kmhIn4,
                  in Boolean limiter_bIn5,
                  in Double leverUp_StatIn6,
                  in Double leverDown_StatIn7,
                  in Boolean fTS_active_bIn8,
                  out Double v_CC_delta_kmhOut1,
                  out Boolean cC_active_bOut2,
                  in Double in1,
                  in Double in2,
                  in Double in3,
                  in Double in4,
                  in Double in5,
                  in Double in6,
                  in Double in7,
                  in Double in8,
                  in Double in9,
                  in Double in10,
                  in Double in11,
                  in Double in12,
                  in Double in13,
                  in Double in14,
                  in Double in15,
                  in Double in16,
                  in Double in17,
                  in Double in18,
                  in Double in19,
                  in Double in20,
                  in Double in21,
                  in Double in22,
                  in Double in23,
                  in Double in24,
                  in Double in25,
                  in Double in26,
                  in Double in27,
                  in Double in28,
                  in Double in29,
                  in Double in30,
                  in Double in31,
                  in Double in32,
                  in Double in33,
                  in Double in34,
                  in Double in35,
                  in Double in36,
                  in Double in37,
                  out Double out1,
                  in Double in38,
                  in Double in39,
                  in Double in40,
                  in Double in41,
                  in Double in42,
                  in Double in43,
                  in Double in44,
                  in Double in45,
                  in Double in46,
                  in Double in47,
                  out Double out2;
                <<Type="SubSystem">> component CC_Enabled {
                  ports
                    in Boolean cC_active_bIn1,
                    in Double v_Vehicle_kmhIn2,
                    out Double v_CC_delta_kmhOut1,
                    in Double in1;
                  <<Type="SubSystem">> component Tempomat_Active {
                    ports
                      in Double v_Vehicle_kmhIn1,
                      out Double v_CC_delta_kmhOut1,
                      in Double in1;
                    <<Type="Sum",ListOfSigns="-+">> component Sum {
                      ports
                        in Double in1,
                        in Double in2,
                        out Double v_CC_delta_kmhOut1;
                      effect in1 -> v_CC_delta_kmhOut1;
                      effect in2 -> v_CC_delta_kmhOut1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Sum sum;
                    component VERSION_INFO vERSION_INFO;
                    connect in1 -> sum.in2;
                    connect sum.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
                    connect v_Vehicle_kmhIn1 -> sum.in1;
                  }
                  <<Type="SubSystem">> component Tempomat_Deactive {
                    ports
                      out Double v_CC_delta_kmhOut1;
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double v_CC_delta_kmhOut1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Constant constant;
                    component VERSION_INFO vERSION_INFO;
                    connect constant.v_CC_delta_kmhOut1 -> v_CC_delta_kmhOut1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  <<Type="Switch">> component SwitchBlock {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1",Type="Condition">> component Condition {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock1 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="!(u1)",Type="Condition">> component Condition1 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                    ports
                      in Double valueIn,
                      out Double valueOut;
                  }
                  component Tempomat_Active tempomat_Active;
                  component Tempomat_Deactive tempomat_Deactive;
                  component VERSION_INFO vERSION_INFO;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  component SwitchBlock1 switchBlock1;
                  component Condition1 condition1;
                  component UnitDelay unitDelay;
                  connect switchBlock.out1 -> v_CC_delta_kmhOut1;
                  connect in1 -> tempomat_Active.in1;
                  connect cC_active_bIn1 -> condition.in1;
                  connect condition.out1 -> switchBlock.condition;
                  connect tempomat_Active.v_CC_delta_kmhOut1 -> switchBlock.ifIn;
                  connect cC_active_bIn1 -> condition1.in1;
                  connect condition1.out1 -> switchBlock1.condition;
                  connect tempomat_Deactive.v_CC_delta_kmhOut1 -> switchBlock1.ifIn;
                  connect switchBlock1.out1 -> switchBlock.elseIn;
                  connect switchBlock.out1 -> unitDelay.valueIn;
                  connect unitDelay.valueOut -> switchBlock1.elseIn;
                  connect v_Vehicle_kmhIn2 -> tempomat_Active.v_Vehicle_kmhIn1;
                }
                <<Type="SubSystem">> component CC_On_Off {
                  ports
                    in Boolean parkingBrake_bIn1,
                    in Double brakeForce_pedal_pcIn2,
                    in Boolean cruiseControl_bIn3,
                    in Double v_Vehicle_kmhIn4,
                    in Boolean limiter_bIn5,
                    in Double leverUp_statIn6,
                    in Double leverDown_statIn7,
                    in Boolean fTS_active_bIn8,
                    out Boolean cC_active_bOut1;
                  <<Type="Constant",Value="0">> component Constant {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="20">> component Constant1 {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="0">> component Constant2 {
                    ports
                      out Double out1;
                  }
                  <<Type="Constant",Value="0">> component Constant3 {
                    ports
                      out Double out1;
                  }
                  <<Type="SubSystem">> component EdgeFalling {
                    ports
                      in Boolean uIn1,
                      in Boolean rIn2,
                      in Boolean iVIn3,
                      out Boolean yOut1;
                    <<Operator="AND",Type="Logic">> component LogOp_A {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_U {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_A logOp_A;
                    component LogOp_N logOp_N;
                    component Memory_U memory_U;
                    component Switch_R switch_R;
                    component Condition condition;
                    connect condition.out1 -> switch_R.condition;
                    connect rIn2 -> condition.in1;
                    connect switch_R.out1 -> logOp_A.in2;
                    connect uIn1 -> logOp_N.in1;
                    connect uIn1 -> memory_U.in1;
                    connect logOp_N.out1 -> logOp_A.in1;
                    connect logOp_A.out1 -> yOut1;
                    connect memory_U.out1 -> switch_R.elseIn;
                    connect iVIn3 -> switch_R.ifIn;
                  }
                  <<Type="SubSystem">> component EdgeRising1 {
                    ports
                      in Boolean uIn1,
                      in Boolean rIn2,
                      in Boolean iVIn3,
                      out Boolean yOut1;
                    <<Operator="AND",Type="Logic">> component LogOp_A {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_U {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_A logOp_A;
                    component LogOp_N logOp_N;
                    component Memory_U memory_U;
                    component Switch_R switch_R;
                    component Condition condition;
                    connect condition.out1 -> switch_R.condition;
                    connect rIn2 -> condition.in1;
                    connect iVIn3 -> switch_R.ifIn;
                    connect memory_U.out1 -> switch_R.elseIn;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect uIn1 -> memory_U.in1;
                    connect uIn1 -> logOp_A.in1;
                    connect logOp_A.out1 -> yOut1;
                    connect logOp_N.out1 -> logOp_A.in2;
                  }
                  <<Type="SubSystem">> component FalseBlock1 {
                    ports
                      out Boolean yOut1;
                    <<Type="Constant",Value="0">> component Zero {
                      ports
                        out Boolean out1;
                    }
                    component Zero zero;
                    connect zero.out1 -> yOut1;
                  }
                  <<Type="SubSystem">> component FalseBlock2 {
                    ports
                      out Boolean yOut1;
                    <<Type="Constant",Value="0">> component Zero {
                      ports
                        out Boolean out1;
                    }
                    component Zero zero;
                    connect zero.out1 -> yOut1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      in Boolean in3,
                      in Boolean in4,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                    effect in3 -> out1;
                    effect in4 -> out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator3 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator4 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator5 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator6 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator7 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Type="SubSystem">> component RSFlipFlop {
                    ports
                      in Boolean sIn1,
                      in Boolean rIn2,
                      out Boolean qOut1,
                      out Boolean nOT_QOut2;
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_Q {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Constant",Value="1">> component One_S {
                      ports
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_S {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Constant",Value="0">> component Zero_R {
                      ports
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_N logOp_N;
                    component Memory_Q memory_Q;
                    component One_S one_S;
                    component Switch_R switch_R;
                    component Switch_S switch_S;
                    component Zero_R zero_R;
                    component Condition condition;
                    component Condition1 condition1;
                    connect condition.out1 -> switch_S.condition;
                    connect condition1.out1 -> switch_R.condition;
                    connect sIn1 -> condition.in1;
                    connect rIn2 -> condition1.in1;
                    connect memory_Q.out1 -> switch_S.elseIn;
                    connect switch_S.out1 -> switch_R.elseIn;
                    connect one_S.out1 -> switch_S.ifIn;
                    connect switch_R.out1 -> memory_Q.in1;
                    connect switch_R.out1 -> qOut1;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect zero_R.out1 -> switch_R.ifIn;
                    connect logOp_N.out1 -> nOT_QOut2;
                  }
                  <<Operator=">",Type="RelationalOperator">> component RelOp {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="<",Type="RelationalOperator">> component RelOp1 {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator=">",Type="RelationalOperator">> component RelOp2 {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator=">",Type="RelationalOperator">> component RelOp3 {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Type="Terminator">> component Terminator {
                    ports
                      in Boolean in1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  component Constant constant;
                  component Constant1 constant1;
                  component Constant2 constant2;
                  component Constant3 constant3;
                  component EdgeFalling edgeFalling;
                  component EdgeRising1 edgeRising1;
                  component FalseBlock1 falseBlock1;
                  component FalseBlock2 falseBlock2;
                  component LogicalOperator logicalOperator;
                  component LogicalOperator1 logicalOperator1;
                  component LogicalOperator2 logicalOperator2;
                  component LogicalOperator3 logicalOperator3;
                  component LogicalOperator4 logicalOperator4;
                  component LogicalOperator5 logicalOperator5;
                  component LogicalOperator6 logicalOperator6;
                  component LogicalOperator7 logicalOperator7;
                  component RSFlipFlop rSFlipFlop;
                  component RelOp relOp;
                  component RelOp1 relOp1;
                  component RelOp2 relOp2;
                  component RelOp3 relOp3;
                  component Terminator terminator;
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
                <<Type="SubSystem">> component CC_SetValue {
                  ports
                    in Boolean limiter_bIn1,
                    in Boolean cC_enabled_bIn2,
                    in Boolean cruiseControl_bIn3,
                    in Double v_Vehicle_kmhIn4,
                    in Double leverUp_StatIn5,
                    in Double leverDown_StatIn6,
                    in Double in1,
                    in Double in2,
                    in Double in3,
                    in Double in4,
                    in Double in5,
                    in Double in6,
                    in Double in7,
                    in Double in8,
                    in Double in9,
                    in Double in10,
                    in Double in11,
                    in Double in12,
                    in Double in13,
                    in Double in14,
                    in Double in15,
                    in Double in16,
                    in Double in17,
                    in Double in18,
                    in Double in19,
                    in Double in20,
                    in Double in21,
                    in Double in22,
                    in Double in23,
                    in Double in24,
                    in Double in25,
                    in Double in26,
                    in Double in27,
                    out Double out1,
                    in Double in28,
                    in Double in29,
                    in Double in30,
                    in Double in31,
                    in Double in32,
                    in Double in33,
                    in Double in34,
                    in Double in35,
                    in Double in36,
                    in Double in37,
                    out Double out2,
                    in Double in38,
                    in Double in39,
                    in Double in40,
                    in Double in41,
                    in Double in42,
                    in Double in43,
                    in Double in44,
                    in Double in45,
                    in Double in46,
                    in Double in47,
                    out Double out3;
                  <<Type="SubSystem">> component CC_ChangeSetValue {
                    ports
                      in Double leverUp_statIn1,
                      in Double leverDown_statIn2,
                      in Double in1,
                      in Double in2,
                      in Double in3,
                      in Double in4,
                      in Double in5,
                      in Double in6,
                      in Double in7,
                      in Double in8,
                      in Double in9,
                      in Double in10,
                      in Double in11,
                      in Double in12,
                      in Double in13,
                      in Double in14,
                      in Double in15,
                      in Double in16,
                      out Double out1,
                      in Double in17,
                      in Double in18,
                      in Double in19,
                      in Double in20,
                      out Double out2,
                      in Double in21,
                      in Double in22,
                      in Double in23,
                      in Double in24,
                      out Double out3,
                      in Double in25,
                      in Double in26,
                      in Double in27,
                      in Double in28,
                      out Double out4,
                      in Double in29,
                      in Double in30,
                      in Double in31,
                      in Double in32,
                      out Double out5,
                      in Double in33,
                      in Double in34,
                      in Double in35,
                      in Double in36,
                      out Double out6,
                      in Double in37,
                      in Double in38,
                      in Double in39,
                      in Double in40,
                      out Double out7;
                    <<Type="SubSystem">> component CC_ChangeSetValue_Lvl2_Repeater {
                      ports
                        in Double leverUp_StatIn1,
                        in Double leverDown_StatIn2,
                        in Double in1,
                        in Double in2,
                        in Double in3,
                        in Double in4,
                        in Double in5,
                        out Double out1,
                        in Double in6,
                        out Double out2,
                        in Double in7,
                        out Double out3,
                        in Double in8,
                        out Double out4,
                        in Double in9,
                        out Double out5,
                        in Double in10,
                        out Double out6,
                        in Double in11,
                        out Double out7;
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant1 {
                        ports
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="1">> component Constant2 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="1">> component Constant3 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="0">> component Constant4 {
                        ports
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="2">> component Constant5 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="2">> component Constant6 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="0">> component Constant7 {
                        ports
                          out Boolean out1;
                      }
                      <<Type="SubSystem">> component CountDown_RE {
                        ports
                          in Boolean eIn1,
                          in Boolean rIn2,
                          in Double iVIn3,
                          out Boolean yOut1;
                        <<Operator="AND",Type="Logic">> component LogOp_C {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="UnitDelay",InitialCondition="0">> component Memory_C {
                          ports
                            in Double in1,
                            out Double out1;
                          effect in1 -> out1;
                        }
                        <<Type="Constant",Value="1">> component One_C {
                          ports
                            out Double out1;
                        }
                        <<Operator=">",Type="RelationalOperator">> component RelOp_C {
                          ports
                            in Double in1,
                            in Double in2,
                            out Boolean out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Operator=">",Type="RelationalOperator">> component RelOp_Y {
                          ports
                            in Double in1,
                            in Double in2,
                            out Boolean out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="+-">> component Sum_C {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_C {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="Constant",Value="0">> component Zero_C {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="2.2204e-11">> component Zero_Y {
                          ports
                            out Double out1;
                        }
                        <<Condition="u2 >= 1",Type="Condition">> component Condition {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        component LogOp_C logOp_C;
                        component Memory_C memory_C;
                        component One_C one_C;
                        component RelOp_C relOp_C;
                        component RelOp_Y relOp_Y;
                        component Sum_C sum_C;
                        component Switch_C switch_C;
                        component Switch_R switch_R;
                        component Zero_C zero_C;
                        component Zero_Y zero_Y;
                        component Condition condition;
                        component Condition1 condition1;
                        connect condition1.out1 -> switch_R.condition;
                        connect condition.out1 -> switch_C.condition;
                        connect logOp_C.out1 -> condition.in1;
                        connect rIn2 -> condition1.in1;
                        connect relOp_Y.out1 -> yOut1;
                        connect zero_Y.out1 -> relOp_Y.in2;
                        connect iVIn3 -> switch_R.ifIn;
                        connect eIn1 -> logOp_C.in1;
                        connect relOp_C.out1 -> logOp_C.in2;
                        connect switch_R.out1 -> memory_C.in1;
                        connect switch_R.out1 -> relOp_Y.in1;
                        connect zero_C.out1 -> relOp_C.in2;
                        connect switch_C.out1 -> switch_R.elseIn;
                        connect one_C.out1 -> sum_C.in2;
                        connect sum_C.out1 -> switch_C.ifIn;
                        connect memory_C.out1 -> relOp_C.in1;
                        connect memory_C.out1 -> sum_C.in1;
                        connect memory_C.out1 -> switch_C.elseIn;
                      }
                      <<Type="Constant",Value="30">> component DEMO_FAS_Repeater_Time {
                        ports
                          out Double out1;
                      }
                      <<Operator="NOT",Type="Logic">> component LogicalOperator {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                        effect in1 -> out1;
                      }
                      <<Operator="OR",Type="Logic">> component LogicalOperator1 {
                        ports
                          in Boolean in1,
                          in Boolean in2,
                          in Boolean in3,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                        effect in3 -> out1;
                      }
                      <<Operator="NOT",Type="Logic">> component LogicalOperator2 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                        effect in1 -> out1;
                      }
                      <<Operator="OR",Type="Logic">> component LogicalOperator3 {
                        ports
                          in Boolean in1,
                          in Boolean in2,
                          in Boolean in3,
                          in Boolean in4,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                        effect in3 -> out1;
                        effect in4 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp1 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp2 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp3 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp4 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                        ports
                          in Boolean ifIn,
                          in Boolean condition,
                          in Boolean elseIn,
                          out Boolean out1;
                        effect ifIn -> out1;
                        effect condition -> out1;
                        effect elseIn -> out1;
                      }
                      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock1 {
                        ports
                          in Boolean ifIn,
                          in Boolean condition,
                          in Boolean elseIn,
                          out Boolean out1;
                        effect ifIn -> out1;
                        effect condition -> out1;
                        effect elseIn -> out1;
                      }
                      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock2 {
                        ports
                          in Boolean ifIn,
                          in Boolean condition,
                          in Boolean elseIn,
                          out Boolean out1;
                        effect ifIn -> out1;
                        effect condition -> out1;
                        effect elseIn -> out1;
                      }
                      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock3 {
                        ports
                          in Boolean ifIn,
                          in Boolean condition,
                          in Boolean elseIn,
                          out Boolean out1;
                        effect ifIn -> out1;
                        effect condition -> out1;
                        effect elseIn -> out1;
                      }
                      <<Type="SubSystem">> component SysInit {
                        ports
                          out Boolean yOut1;
                        <<Type="UnitDelay",InitialCondition="1">> component Memory_Init {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                          effect in1 -> out1;
                        }
                        <<Type="Constant",Value="0">> component Zero_Init {
                          ports
                            out Boolean out1;
                        }
                        component Memory_Init memory_Init;
                        component Zero_Init zero_Init;
                        connect memory_Init.out1 -> yOut1;
                        connect zero_Init.out1 -> memory_Init.in1;
                      }
                      <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                        effect in1 -> out1;
                      }
                      <<Type="SubSystem">> component VERSION_INFO {
                        <<Type="SubSystem">> component Copyright {
                        }
                        component Copyright copyright;
                      }
                      <<Type="SubSystem">> component V_SetValueMinus {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7;
                        <<Type="Constant",Value="5">> component Parameter {
                          ports
                            out Double out1;
                        }
                        <<Type="Sum",ListOfSigns="+-">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        component Parameter parameter;
                        component Sum sum;
                        component VERSION_INFO vERSION_INFO;
                        connect sum.out1 -> out1;
                        connect sum.out1 -> out2;
                        connect sum.out1 -> out3;
                        connect sum.out1 -> out4;
                        connect sum.out1 -> out5;
                        connect sum.out1 -> out6;
                        connect sum.out1 -> out7;
                        connect parameter.out1 -> sum.in2;
                        connect v_CCSetValueIn1 -> sum.in1;
                      }
                      <<Type="SubSystem">> component V_SetValueMinusLvl2 {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7;
                        <<Type="Constant",Value="10">> component Constant1 {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="10">> component DEMO_FAS_CC_Lvl2_Round {
                          ports
                            out Double out1;
                        }
                        <<Function="mod",Type="Math">> component MathFunction {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="+-">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="DEMO_FAS_CC_Min_Value">> component SwitchBlock1 {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="Switch",Criteria="u2 ~= 0",Threshold="1">> component SwitchBlock3 {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        <<Condition="u2 >= DEMO_FAS_CC_Min_Value",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        <<Condition="u2 ~= 0",Type="Condition">> component Condition1 {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component Constant1 constant1;
                        component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;
                        component MathFunction mathFunction;
                        component Sum sum;
                        component SwitchBlock1 switchBlock1;
                        component SwitchBlock3 switchBlock3;
                        component VERSION_INFO vERSION_INFO;
                        component Condition condition;
                        component Condition1 condition1;
                        connect condition1.out1 -> switchBlock3.condition;
                        connect condition.out1 -> switchBlock1.condition;
                        connect switchBlock1.out1 -> out1;
                        connect switchBlock1.out1 -> out2;
                        connect switchBlock1.out1 -> out3;
                        connect switchBlock1.out1 -> out4;
                        connect switchBlock1.out1 -> out5;
                        connect switchBlock1.out1 -> out6;
                        connect switchBlock1.out1 -> out7;
                        connect sum.out1 -> condition.in1;
                        connect mathFunction.out1 -> condition1.in1;
                        connect switchBlock3.out1 -> sum.in2;
                        connect constant1.out1 -> switchBlock3.elseIn;
                        connect mathFunction.out1 -> switchBlock3.ifIn;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
                        connect v_CCSetValueIn1 -> mathFunction.in1;
                        connect v_CCSetValueIn1 -> sum.in1;
                        connect v_CCSetValueIn1 -> switchBlock1.elseIn;
                        connect sum.out1 -> switchBlock1.ifIn;
                      }
                      <<Type="SubSystem">> component V_SetValuePlus {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7;
                        <<Type="Constant",Value="5">> component Parameter {
                          ports
                            out Double out1;
                        }
                        <<Type="Sum",ListOfSigns="++">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        component Parameter parameter;
                        component Sum sum;
                        component VERSION_INFO vERSION_INFO;
                        connect sum.out1 -> out1;
                        connect sum.out1 -> out2;
                        connect sum.out1 -> out3;
                        connect sum.out1 -> out4;
                        connect sum.out1 -> out5;
                        connect sum.out1 -> out6;
                        connect sum.out1 -> out7;
                        connect parameter.out1 -> sum.in2;
                        connect v_CCSetValueIn1 -> sum.in1;
                      }
                      <<Type="SubSystem">> component V_SetValuePlusLvl2 {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7;
                        <<Type="Constant",Value="10">> component DEMO_FAS_CC_Lvl2_Round {
                          ports
                            out Double out1;
                        }
                        <<Function="mod",Type="Math">> component MathFunction {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="++">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="-+">> component Sum1 {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;
                        component MathFunction mathFunction;
                        component Sum sum;
                        component Sum1 sum1;
                        component VERSION_INFO vERSION_INFO;
                        connect sum.out1 -> out1;
                        connect sum.out1 -> out2;
                        connect sum.out1 -> out3;
                        connect sum.out1 -> out4;
                        connect sum.out1 -> out5;
                        connect sum.out1 -> out6;
                        connect sum.out1 -> out7;
                        connect sum1.out1 -> sum.in2;
                        connect mathFunction.out1 -> sum1.in1;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> sum1.in2;
                        connect v_CCSetValueIn1 -> mathFunction.in1;
                        connect v_CCSetValueIn1 -> sum.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector1 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector2 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector3 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="Switch">> component SwitchBlock4 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock5 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition1 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock6 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition2 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock7 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition3 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock8 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition4 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock9 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition5 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock10 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition6 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock11 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition7 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock12 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition8 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock13 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition9 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock14 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition10 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock15 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition11 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock16 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition12 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock17 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition13 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock18 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition14 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock19 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition15 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock20 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition16 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock21 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition17 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock22 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition18 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock23 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition19 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock24 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition20 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock25 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition21 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock26 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition22 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock27 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition23 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock28 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition24 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock29 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition25 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock30 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition26 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock31 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition27 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Condition="u2 >= 1",Type="Condition">> component Condition28 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Condition="u2 >= 1",Type="Condition">> component Condition29 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Condition="u2 >= 1",Type="Condition">> component Condition30 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Condition="u2 >= 1",Type="Condition">> component Condition31 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      component Constant constant;
                      component Constant1 constant1;
                      component Constant2 constant2;
                      component Constant3 constant3;
                      component Constant4 constant4;
                      component Constant5 constant5;
                      component Constant6 constant6;
                      component Constant7 constant7;
                      component CountDown_RE countDown_RE;
                      component DEMO_FAS_Repeater_Time dEMO_FAS_Repeater_Time;
                      component LogicalOperator logicalOperator;
                      component LogicalOperator1 logicalOperator1;
                      component LogicalOperator2 logicalOperator2;
                      component LogicalOperator3 logicalOperator3;
                      component RelOp1 relOp1;
                      component RelOp2 relOp2;
                      component RelOp3 relOp3;
                      component RelOp4 relOp4;
                      component SwitchBlock switchBlock;
                      component SwitchBlock1 switchBlock1;
                      component SwitchBlock2 switchBlock2;
                      component SwitchBlock3 switchBlock3;
                      component SysInit sysInit;
                      component UnitDelay unitDelay;
                      component VERSION_INFO vERSION_INFO;
                      component V_SetValueMinus v_SetValueMinus;
                      component V_SetValueMinusLvl2 v_SetValueMinusLvl2;
                      component V_SetValuePlus v_SetValuePlus;
                      component V_SetValuePlusLvl2 v_SetValuePlusLvl2;
                      component RisingEdgeDetector risingEdgeDetector;
                      component RisingEdgeDetector1 risingEdgeDetector1;
                      component RisingEdgeDetector2 risingEdgeDetector2;
                      component RisingEdgeDetector3 risingEdgeDetector3;
                      component SwitchBlock4 switchBlock4;
                      component Condition condition;
                      component SwitchBlock5 switchBlock5;
                      component Condition1 condition1;
                      component SwitchBlock6 switchBlock6;
                      component Condition2 condition2;
                      component SwitchBlock7 switchBlock7;
                      component Condition3 condition3;
                      component SwitchBlock8 switchBlock8;
                      component Condition4 condition4;
                      component SwitchBlock9 switchBlock9;
                      component Condition5 condition5;
                      component SwitchBlock10 switchBlock10;
                      component Condition6 condition6;
                      component SwitchBlock11 switchBlock11;
                      component Condition7 condition7;
                      component SwitchBlock12 switchBlock12;
                      component Condition8 condition8;
                      component SwitchBlock13 switchBlock13;
                      component Condition9 condition9;
                      component SwitchBlock14 switchBlock14;
                      component Condition10 condition10;
                      component SwitchBlock15 switchBlock15;
                      component Condition11 condition11;
                      component SwitchBlock16 switchBlock16;
                      component Condition12 condition12;
                      component SwitchBlock17 switchBlock17;
                      component Condition13 condition13;
                      component SwitchBlock18 switchBlock18;
                      component Condition14 condition14;
                      component SwitchBlock19 switchBlock19;
                      component Condition15 condition15;
                      component SwitchBlock20 switchBlock20;
                      component Condition16 condition16;
                      component SwitchBlock21 switchBlock21;
                      component Condition17 condition17;
                      component SwitchBlock22 switchBlock22;
                      component Condition18 condition18;
                      component SwitchBlock23 switchBlock23;
                      component Condition19 condition19;
                      component SwitchBlock24 switchBlock24;
                      component Condition20 condition20;
                      component SwitchBlock25 switchBlock25;
                      component Condition21 condition21;
                      component SwitchBlock26 switchBlock26;
                      component Condition22 condition22;
                      component SwitchBlock27 switchBlock27;
                      component Condition23 condition23;
                      component SwitchBlock28 switchBlock28;
                      component Condition24 condition24;
                      component SwitchBlock29 switchBlock29;
                      component Condition25 condition25;
                      component SwitchBlock30 switchBlock30;
                      component Condition26 condition26;
                      component SwitchBlock31 switchBlock31;
                      component Condition27 condition27;
                      component Condition28 condition28;
                      component Condition29 condition29;
                      component Condition30 condition30;
                      component Condition31 condition31;
                      connect condition29.out1 -> switchBlock.condition;
                      connect condition31.out1 -> switchBlock2.condition;
                      connect condition28.out1 -> switchBlock3.condition;
                      connect condition30.out1 -> switchBlock1.condition;
                      connect in2 -> v_SetValuePlus.v_CCSetValueIn1;
                      connect in4 -> v_SetValuePlusLvl2.v_CCSetValueIn1;
                      connect in1 -> v_SetValueMinusLvl2.v_CCSetValueIn1;
                      connect in3 -> v_SetValueMinus.v_CCSetValueIn1;
                      connect switchBlock.out1 -> risingEdgeDetector.in1In1;
                      connect switchBlock3.out1 -> risingEdgeDetector1.in1In1;
                      connect switchBlock1.out1 -> risingEdgeDetector2.in1In1;
                      connect switchBlock2.out1 -> risingEdgeDetector3.in1In1;
                      connect risingEdgeDetector3.out1Out1 -> condition.in1;
                      connect v_SetValuePlusLvl2.out1 -> switchBlock4.ifIn;
                      connect condition.out1 -> switchBlock4.condition;
                      connect risingEdgeDetector.out1Out1 -> condition1.in1;
                      connect v_SetValuePlus.out1 -> switchBlock5.ifIn;
                      connect condition1.out1 -> switchBlock5.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition2.in1;
                      connect v_SetValueMinusLvl2.out1 -> switchBlock6.ifIn;
                      connect condition2.out1 -> switchBlock6.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition3.in1;
                      connect v_SetValueMinus.out1 -> switchBlock7.ifIn;
                      connect condition3.out1 -> switchBlock7.condition;
                      connect in5 -> switchBlock7.elseIn;
                      connect switchBlock7.out1 -> switchBlock6.elseIn;
                      connect switchBlock6.out1 -> switchBlock5.elseIn;
                      connect switchBlock5.out1 -> switchBlock4.elseIn;
                      connect switchBlock4.out1 -> out1;
                      connect risingEdgeDetector3.out1Out1 -> condition4.in1;
                      connect v_SetValuePlusLvl2.out2 -> switchBlock8.ifIn;
                      connect condition4.out1 -> switchBlock8.condition;
                      connect risingEdgeDetector.out1Out1 -> condition5.in1;
                      connect v_SetValuePlus.out2 -> switchBlock9.ifIn;
                      connect condition5.out1 -> switchBlock9.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition6.in1;
                      connect v_SetValueMinusLvl2.out2 -> switchBlock10.ifIn;
                      connect condition6.out1 -> switchBlock10.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition7.in1;
                      connect v_SetValueMinus.out2 -> switchBlock11.ifIn;
                      connect condition7.out1 -> switchBlock11.condition;
                      connect in6 -> switchBlock11.elseIn;
                      connect switchBlock11.out1 -> switchBlock10.elseIn;
                      connect switchBlock10.out1 -> switchBlock9.elseIn;
                      connect switchBlock9.out1 -> switchBlock8.elseIn;
                      connect switchBlock8.out1 -> out2;
                      connect risingEdgeDetector3.out1Out1 -> condition8.in1;
                      connect v_SetValuePlusLvl2.out3 -> switchBlock12.ifIn;
                      connect condition8.out1 -> switchBlock12.condition;
                      connect risingEdgeDetector.out1Out1 -> condition9.in1;
                      connect v_SetValuePlus.out3 -> switchBlock13.ifIn;
                      connect condition9.out1 -> switchBlock13.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition10.in1;
                      connect v_SetValueMinusLvl2.out3 -> switchBlock14.ifIn;
                      connect condition10.out1 -> switchBlock14.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition11.in1;
                      connect v_SetValueMinus.out3 -> switchBlock15.ifIn;
                      connect condition11.out1 -> switchBlock15.condition;
                      connect in7 -> switchBlock15.elseIn;
                      connect switchBlock15.out1 -> switchBlock14.elseIn;
                      connect switchBlock14.out1 -> switchBlock13.elseIn;
                      connect switchBlock13.out1 -> switchBlock12.elseIn;
                      connect switchBlock12.out1 -> out3;
                      connect risingEdgeDetector3.out1Out1 -> condition12.in1;
                      connect v_SetValuePlusLvl2.out4 -> switchBlock16.ifIn;
                      connect condition12.out1 -> switchBlock16.condition;
                      connect risingEdgeDetector.out1Out1 -> condition13.in1;
                      connect v_SetValuePlus.out4 -> switchBlock17.ifIn;
                      connect condition13.out1 -> switchBlock17.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition14.in1;
                      connect v_SetValueMinusLvl2.out4 -> switchBlock18.ifIn;
                      connect condition14.out1 -> switchBlock18.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition15.in1;
                      connect v_SetValueMinus.out4 -> switchBlock19.ifIn;
                      connect condition15.out1 -> switchBlock19.condition;
                      connect in8 -> switchBlock19.elseIn;
                      connect switchBlock19.out1 -> switchBlock18.elseIn;
                      connect switchBlock18.out1 -> switchBlock17.elseIn;
                      connect switchBlock17.out1 -> switchBlock16.elseIn;
                      connect switchBlock16.out1 -> out4;
                      connect risingEdgeDetector3.out1Out1 -> condition16.in1;
                      connect v_SetValuePlusLvl2.out5 -> switchBlock20.ifIn;
                      connect condition16.out1 -> switchBlock20.condition;
                      connect risingEdgeDetector.out1Out1 -> condition17.in1;
                      connect v_SetValuePlus.out5 -> switchBlock21.ifIn;
                      connect condition17.out1 -> switchBlock21.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition18.in1;
                      connect v_SetValueMinusLvl2.out5 -> switchBlock22.ifIn;
                      connect condition18.out1 -> switchBlock22.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition19.in1;
                      connect v_SetValueMinus.out5 -> switchBlock23.ifIn;
                      connect condition19.out1 -> switchBlock23.condition;
                      connect in9 -> switchBlock23.elseIn;
                      connect switchBlock23.out1 -> switchBlock22.elseIn;
                      connect switchBlock22.out1 -> switchBlock21.elseIn;
                      connect switchBlock21.out1 -> switchBlock20.elseIn;
                      connect switchBlock20.out1 -> out5;
                      connect risingEdgeDetector3.out1Out1 -> condition20.in1;
                      connect v_SetValuePlusLvl2.out6 -> switchBlock24.ifIn;
                      connect condition20.out1 -> switchBlock24.condition;
                      connect risingEdgeDetector.out1Out1 -> condition21.in1;
                      connect v_SetValuePlus.out6 -> switchBlock25.ifIn;
                      connect condition21.out1 -> switchBlock25.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition22.in1;
                      connect v_SetValueMinusLvl2.out6 -> switchBlock26.ifIn;
                      connect condition22.out1 -> switchBlock26.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition23.in1;
                      connect v_SetValueMinus.out6 -> switchBlock27.ifIn;
                      connect condition23.out1 -> switchBlock27.condition;
                      connect in10 -> switchBlock27.elseIn;
                      connect switchBlock27.out1 -> switchBlock26.elseIn;
                      connect switchBlock26.out1 -> switchBlock25.elseIn;
                      connect switchBlock25.out1 -> switchBlock24.elseIn;
                      connect switchBlock24.out1 -> out6;
                      connect risingEdgeDetector3.out1Out1 -> condition24.in1;
                      connect v_SetValuePlusLvl2.out7 -> switchBlock28.ifIn;
                      connect condition24.out1 -> switchBlock28.condition;
                      connect risingEdgeDetector.out1Out1 -> condition25.in1;
                      connect v_SetValuePlus.out7 -> switchBlock29.ifIn;
                      connect condition25.out1 -> switchBlock29.condition;
                      connect risingEdgeDetector1.out1Out1 -> condition26.in1;
                      connect v_SetValueMinusLvl2.out7 -> switchBlock30.ifIn;
                      connect condition26.out1 -> switchBlock30.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition27.in1;
                      connect v_SetValueMinus.out7 -> switchBlock31.ifIn;
                      connect condition27.out1 -> switchBlock31.condition;
                      connect in11 -> switchBlock31.elseIn;
                      connect switchBlock31.out1 -> switchBlock30.elseIn;
                      connect switchBlock30.out1 -> switchBlock29.elseIn;
                      connect switchBlock29.out1 -> switchBlock28.elseIn;
                      connect switchBlock28.out1 -> out7;
                      connect logicalOperator2.out1 -> condition28.in1;
                      connect logicalOperator2.out1 -> condition29.in1;
                      connect logicalOperator2.out1 -> condition30.in1;
                      connect logicalOperator2.out1 -> condition31.in1;
                      connect constant7.out1 -> switchBlock3.ifIn;
                      connect logicalOperator2.out1 -> unitDelay.in1;
                      connect relOp4.out1 -> switchBlock3.elseIn;
                      connect relOp4.out1 -> logicalOperator3.in1;
                      connect leverDown_StatIn2 -> relOp1.in1;
                      connect leverDown_StatIn2 -> relOp4.in1;
                      connect constant6.out1 -> relOp4.in2;
                      connect leverUp_StatIn1 -> relOp3.in1;
                      connect leverUp_StatIn1 -> relOp2.in1;
                      connect constant5.out1 -> relOp2.in2;
                      connect constant4.out1 -> switchBlock2.ifIn;
                      connect relOp2.out1 -> switchBlock2.elseIn;
                      connect relOp2.out1 -> logicalOperator3.in4;
                      connect constant2.out1 -> relOp1.in2;
                      connect constant3.out1 -> relOp3.in2;
                      connect sysInit.yOut1 -> logicalOperator1.in3;
                      connect logicalOperator3.out1 -> logicalOperator.in1;
                      connect logicalOperator3.out1 -> countDown_RE.eIn1;
                      connect constant1.out1 -> switchBlock1.ifIn;
                      connect constant.out1 -> switchBlock.ifIn;
                      connect unitDelay.out1 -> logicalOperator1.in1;
                      connect logicalOperator1.out1 -> countDown_RE.rIn2;
                      connect logicalOperator.out1 -> logicalOperator1.in2;
                      connect dEMO_FAS_Repeater_Time.out1 -> countDown_RE.iVIn3;
                      connect countDown_RE.yOut1 -> logicalOperator2.in1;
                      connect relOp3.out1 -> switchBlock.elseIn;
                      connect relOp3.out1 -> logicalOperator3.in3;
                      connect relOp1.out1 -> switchBlock1.elseIn;
                      connect relOp1.out1 -> logicalOperator3.in2;
                    }
                    <<Type="SubSystem">> component CC_ChangeSetValue_Lvl2_no_Repeater {
                      ports
                        in Double leverUp_statIn1,
                        in Double leverDown_statIn2,
                        in Double in1,
                        in Double in2,
                        in Double in3,
                        in Double in4,
                        in Double in5,
                        out Double out1,
                        in Double in6,
                        out Double out2,
                        in Double in7,
                        out Double out3,
                        in Double in8,
                        out Double out4,
                        in Double in9,
                        out Double out5,
                        in Double in10,
                        out Double out6,
                        in Double in11,
                        out Double out7,
                        in Double in12,
                        out Double out8,
                        in Double in13,
                        out Double out9,
                        in Double in14,
                        out Double out10,
                        in Double in15,
                        out Double out11,
                        in Double in16,
                        out Double out12,
                        in Double in17,
                        out Double out13,
                        in Double in18,
                        out Double out14,
                        in Double in19,
                        out Double out15,
                        in Double in20,
                        out Double out16,
                        in Double in21,
                        out Double out17,
                        in Double in22,
                        out Double out18;
                      <<Type="Constant",Value="1">> component Constant1 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="1">> component Constant2 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="2">> component Constant3 {
                        ports
                          out Double out1;
                      }
                      <<Type="Constant",Value="2">> component Constant4 {
                        ports
                          out Double out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp1 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp2 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp3 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Operator="==",Type="RelationalOperator">> component RelOp4 {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                        effect in1 -> out1;
                        effect in2 -> out1;
                      }
                      <<Type="SubSystem">> component VERSION_INFO {
                        <<Type="SubSystem">> component Copyright {
                        }
                        component Copyright copyright;
                      }
                      <<Type="SubSystem">> component V_SetValueMinus {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7,
                          out Double out8,
                          out Double out9,
                          out Double out10,
                          out Double out11,
                          out Double out12,
                          out Double out13,
                          out Double out14,
                          out Double out15,
                          out Double out16,
                          out Double out17,
                          out Double out18;
                        <<Type="Constant",Value="5">> component Parameter {
                          ports
                            out Double out1;
                        }
                        <<Type="Sum",ListOfSigns="+-">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="DEMO_FAS_CC_Min_Value">> component SwitchBlock {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        <<Condition="u2 >= DEMO_FAS_CC_Min_Value",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component Parameter parameter;
                        component Sum sum;
                        component SwitchBlock switchBlock;
                        component VERSION_INFO vERSION_INFO;
                        component Condition condition;
                        connect condition.out1 -> switchBlock.condition;
                        connect switchBlock.out1 -> out1;
                        connect switchBlock.out1 -> out2;
                        connect switchBlock.out1 -> out3;
                        connect switchBlock.out1 -> out4;
                        connect switchBlock.out1 -> out5;
                        connect switchBlock.out1 -> out6;
                        connect switchBlock.out1 -> out7;
                        connect switchBlock.out1 -> out8;
                        connect switchBlock.out1 -> out9;
                        connect switchBlock.out1 -> out10;
                        connect switchBlock.out1 -> out11;
                        connect switchBlock.out1 -> out12;
                        connect switchBlock.out1 -> out13;
                        connect switchBlock.out1 -> out14;
                        connect switchBlock.out1 -> out15;
                        connect switchBlock.out1 -> out16;
                        connect switchBlock.out1 -> out17;
                        connect switchBlock.out1 -> out18;
                        connect sum.out1 -> condition.in1;
                        connect sum.out1 -> switchBlock.ifIn;
                        connect v_CCSetValueIn1 -> sum.in1;
                        connect v_CCSetValueIn1 -> switchBlock.elseIn;
                        connect parameter.out1 -> sum.in2;
                      }
                      <<Type="SubSystem">> component V_SetValueMinusLvl2 {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7,
                          out Double out8,
                          out Double out9,
                          out Double out10,
                          out Double out11,
                          out Double out12,
                          out Double out13,
                          out Double out14,
                          out Double out15,
                          out Double out16,
                          out Double out17,
                          out Double out18;
                        <<Type="Constant",Value="10">> component Constant1 {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="10">> component DEMO_FAS_CC_Lvl2_Round {
                          ports
                            out Double out1;
                        }
                        <<Function="mod",Type="Math">> component MathFunction {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="+-">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="DEMO_FAS_CC_Min_Value">> component SwitchBlock1 {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="Switch",Criteria="u2 ~= 0",Threshold="1">> component SwitchBlock3 {
                          ports
                            in Double ifIn,
                            in Boolean condition,
                            in Double elseIn,
                            out Double out1;
                          effect ifIn -> out1;
                          effect condition -> out1;
                          effect elseIn -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        <<Condition="u2 >= DEMO_FAS_CC_Min_Value",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        <<Condition="u2 ~= 0",Type="Condition">> component Condition1 {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component Constant1 constant1;
                        component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;
                        component MathFunction mathFunction;
                        component Sum sum;
                        component SwitchBlock1 switchBlock1;
                        component SwitchBlock3 switchBlock3;
                        component VERSION_INFO vERSION_INFO;
                        component Condition condition;
                        component Condition1 condition1;
                        connect condition1.out1 -> switchBlock3.condition;
                        connect condition.out1 -> switchBlock1.condition;
                        connect switchBlock1.out1 -> out1;
                        connect switchBlock1.out1 -> out2;
                        connect switchBlock1.out1 -> out3;
                        connect switchBlock1.out1 -> out4;
                        connect switchBlock1.out1 -> out5;
                        connect switchBlock1.out1 -> out6;
                        connect switchBlock1.out1 -> out7;
                        connect switchBlock1.out1 -> out8;
                        connect switchBlock1.out1 -> out9;
                        connect switchBlock1.out1 -> out10;
                        connect switchBlock1.out1 -> out11;
                        connect switchBlock1.out1 -> out12;
                        connect switchBlock1.out1 -> out13;
                        connect switchBlock1.out1 -> out14;
                        connect switchBlock1.out1 -> out15;
                        connect switchBlock1.out1 -> out16;
                        connect switchBlock1.out1 -> out17;
                        connect switchBlock1.out1 -> out18;
                        connect sum.out1 -> condition.in1;
                        connect mathFunction.out1 -> condition1.in1;
                        connect switchBlock3.out1 -> sum.in2;
                        connect constant1.out1 -> switchBlock3.elseIn;
                        connect mathFunction.out1 -> switchBlock3.ifIn;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
                        connect v_CCSetValueIn1 -> mathFunction.in1;
                        connect v_CCSetValueIn1 -> sum.in1;
                        connect v_CCSetValueIn1 -> switchBlock1.elseIn;
                        connect sum.out1 -> switchBlock1.ifIn;
                      }
                      <<Type="SubSystem">> component V_SetValuePlus {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7,
                          out Double out8,
                          out Double out9,
                          out Double out10,
                          out Double out11,
                          out Double out12,
                          out Double out13,
                          out Double out14,
                          out Double out15,
                          out Double out16,
                          out Double out17,
                          out Double out18;
                        <<Type="Constant",Value="5">> component Parameter {
                          ports
                            out Double out1;
                        }
                        <<Type="Sum",ListOfSigns="++">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        component Parameter parameter;
                        component Sum sum;
                        component VERSION_INFO vERSION_INFO;
                        connect sum.out1 -> out1;
                        connect sum.out1 -> out2;
                        connect sum.out1 -> out3;
                        connect sum.out1 -> out4;
                        connect sum.out1 -> out5;
                        connect sum.out1 -> out6;
                        connect sum.out1 -> out7;
                        connect sum.out1 -> out8;
                        connect sum.out1 -> out9;
                        connect sum.out1 -> out10;
                        connect sum.out1 -> out11;
                        connect sum.out1 -> out12;
                        connect sum.out1 -> out13;
                        connect sum.out1 -> out14;
                        connect sum.out1 -> out15;
                        connect sum.out1 -> out16;
                        connect sum.out1 -> out17;
                        connect sum.out1 -> out18;
                        connect parameter.out1 -> sum.in2;
                        connect v_CCSetValueIn1 -> sum.in1;
                      }
                      <<Type="SubSystem">> component V_SetValuePlusLvl2 {
                        ports
                          in Double v_CCSetValueIn1,
                          out Double out1,
                          out Double out2,
                          out Double out3,
                          out Double out4,
                          out Double out5,
                          out Double out6,
                          out Double out7,
                          out Double out8,
                          out Double out9,
                          out Double out10,
                          out Double out11,
                          out Double out12,
                          out Double out13,
                          out Double out14,
                          out Double out15,
                          out Double out16,
                          out Double out17,
                          out Double out18;
                        <<Type="Constant",Value="10">> component DEMO_FAS_CC_Lvl2_Round {
                          ports
                            out Double out1;
                        }
                        <<Function="mod",Type="Math">> component MathFunction {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="++">> component Sum {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="Sum",ListOfSigns="-+">> component Sum1 {
                          ports
                            in Double in1,
                            in Double in2,
                            out Double out1;
                          effect in1 -> out1;
                          effect in2 -> out1;
                        }
                        <<Type="SubSystem">> component VERSION_INFO {
                          <<Type="SubSystem">> component Copyright {
                          }
                          component Copyright copyright;
                        }
                        component DEMO_FAS_CC_Lvl2_Round dEMO_FAS_CC_Lvl2_Round;
                        component MathFunction mathFunction;
                        component Sum sum;
                        component Sum1 sum1;
                        component VERSION_INFO vERSION_INFO;
                        connect sum.out1 -> out1;
                        connect sum.out1 -> out2;
                        connect sum.out1 -> out3;
                        connect sum.out1 -> out4;
                        connect sum.out1 -> out5;
                        connect sum.out1 -> out6;
                        connect sum.out1 -> out7;
                        connect sum.out1 -> out8;
                        connect sum.out1 -> out9;
                        connect sum.out1 -> out10;
                        connect sum.out1 -> out11;
                        connect sum.out1 -> out12;
                        connect sum.out1 -> out13;
                        connect sum.out1 -> out14;
                        connect sum.out1 -> out15;
                        connect sum.out1 -> out16;
                        connect sum.out1 -> out17;
                        connect sum.out1 -> out18;
                        connect sum1.out1 -> sum.in2;
                        connect mathFunction.out1 -> sum1.in1;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> mathFunction.in2;
                        connect dEMO_FAS_CC_Lvl2_Round.out1 -> sum1.in2;
                        connect v_CCSetValueIn1 -> mathFunction.in1;
                        connect v_CCSetValueIn1 -> sum.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector1 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector2 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="SubSystem">> component RisingEdgeDetector3 {
                        ports
                          in Boolean in1In1,
                          out Boolean out1Out1;
                        <<Type="SubSystem">> component CompareToZero {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero1 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero2 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="<",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero3 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator=">=",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="SubSystem">> component CompareToZero4 {
                          ports
                            in Boolean uIn1,
                            out Boolean yOut1;
                          <<Operator="==",Type="RelationalOperator">> component Compare {
                            ports
                              in Boolean in1,
                              in Double in2,
                              out Boolean out1;
                          }
                          <<Type="Constant",Value="0">> component Constant {
                            ports
                              out Double out1;
                          }
                          component Compare compare;
                          component Constant constant;
                          connect compare.out1 -> yOut1;
                          connect uIn1 -> compare.in1;
                          connect constant.out1 -> compare.in2;
                        }
                        <<Type="Constant",Value="1">> component Constant {
                          ports
                            out Double out1;
                        }
                        <<Type="Constant",Value="0">> component Constant1 {
                          ports
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            in Boolean in3,
                            out Boolean out1;
                        }
                        <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                          ports
                            in Boolean in1,
                            in Boolean in2,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Memory">> component Memory1 {
                          ports
                            in Double in1,
                            out Double out1;
                        }
                        <<Type="Memory">> component Memory2 {
                          ports
                            in Boolean in1,
                            out Boolean out1;
                        }
                        <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                          ports
                            in Boolean ifIn,
                            in Boolean condition,
                            in Boolean elseIn,
                            out Boolean out1;
                        }
                        <<Condition="u2 > 0",Type="Condition">> component Condition {
                          ports
                            in Double in1,
                            out Boolean out1;
                        }
                        component CompareToZero compareToZero;
                        component CompareToZero1 compareToZero1;
                        component CompareToZero2 compareToZero2;
                        component CompareToZero3 compareToZero3;
                        component CompareToZero4 compareToZero4;
                        component Constant constant;
                        component Constant1 constant1;
                        component LogicalOperator logicalOperator;
                        component LogicalOperator1 logicalOperator1;
                        component LogicalOperator2 logicalOperator2;
                        component Memory memory;
                        component Memory1 memory1;
                        component Memory2 memory2;
                        component SwitchBlock switchBlock;
                        component Condition condition;
                        connect switchBlock.out1 -> out1Out1;
                        connect compareToZero4.yOut1 -> logicalOperator.in3;
                        connect memory2.out1 -> compareToZero4.uIn1;
                        connect logicalOperator.out1 -> logicalOperator2.in1;
                        connect constant1.out1 -> switchBlock.elseIn;
                        connect constant.out1 -> memory1.in1;
                        connect condition.out1 -> switchBlock.condition;
                        connect logicalOperator2.out1 -> switchBlock.ifIn;
                        connect logicalOperator2.out1 -> memory2.in1;
                        connect logicalOperator1.out1 -> logicalOperator2.in2;
                        connect compareToZero3.yOut1 -> logicalOperator1.in2;
                        connect compareToZero2.yOut1 -> logicalOperator1.in1;
                        connect compareToZero1.yOut1 -> logicalOperator.in2;
                        connect compareToZero.yOut1 -> logicalOperator.in1;
                        connect memory.out1 -> compareToZero.uIn1;
                        connect memory.out1 -> compareToZero2.uIn1;
                        connect in1In1 -> memory.in1;
                        connect in1In1 -> compareToZero1.uIn1;
                        connect in1In1 -> compareToZero3.uIn1;
                        connect memory1.out1 -> condition.in1;
                      }
                      <<Type="Switch">> component SwitchBlock {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock1 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition1 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock2 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition2 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock3 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition3 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock4 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition4 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock5 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition5 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock6 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition6 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock7 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition7 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock8 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition8 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock9 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition9 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock10 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition10 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock11 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition11 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock12 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition12 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock13 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition13 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock14 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition14 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock15 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition15 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock16 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition16 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock17 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition17 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock18 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition18 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock19 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition19 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock20 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition20 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock21 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition21 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock22 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition22 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock23 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition23 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock24 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition24 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock25 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition25 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock26 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition26 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock27 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition27 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock28 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition28 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock29 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition29 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock30 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition30 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock31 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition31 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock32 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition32 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock33 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition33 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock34 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition34 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock35 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition35 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock36 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition36 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock37 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition37 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock38 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition38 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock39 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition39 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock40 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition40 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock41 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition41 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock42 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition42 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock43 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition43 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock44 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition44 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock45 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition45 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock46 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition46 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock47 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition47 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock48 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition48 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock49 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition49 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock50 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition50 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock51 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition51 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock52 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition52 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock53 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition53 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock54 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition54 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock55 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition55 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock56 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition56 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock57 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition57 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock58 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition58 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock59 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition59 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock60 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition60 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock61 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition61 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock62 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition62 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock63 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition63 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock64 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition64 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock65 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition65 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock66 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition66 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock67 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition67 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock68 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition68 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock69 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition69 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock70 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition70 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      <<Type="Switch">> component SwitchBlock71 {
                        ports
                          in Double ifIn,
                          in Boolean condition,
                          in Double elseIn,
                          out Double out1;
                      }
                      <<Condition="u1>0",Type="Condition">> component Condition71 {
                        ports
                          in Boolean in1,
                          out Boolean out1;
                      }
                      component Constant1 constant1;
                      component Constant2 constant2;
                      component Constant3 constant3;
                      component Constant4 constant4;
                      component RelOp1 relOp1;
                      component RelOp2 relOp2;
                      component RelOp3 relOp3;
                      component RelOp4 relOp4;
                      component VERSION_INFO vERSION_INFO;
                      component V_SetValueMinus v_SetValueMinus;
                      component V_SetValueMinusLvl2 v_SetValueMinusLvl2;
                      component V_SetValuePlus v_SetValuePlus;
                      component V_SetValuePlusLvl2 v_SetValuePlusLvl2;
                      component RisingEdgeDetector risingEdgeDetector;
                      component RisingEdgeDetector1 risingEdgeDetector1;
                      component RisingEdgeDetector2 risingEdgeDetector2;
                      component RisingEdgeDetector3 risingEdgeDetector3;
                      component SwitchBlock switchBlock;
                      component Condition condition;
                      component SwitchBlock1 switchBlock1;
                      component Condition1 condition1;
                      component SwitchBlock2 switchBlock2;
                      component Condition2 condition2;
                      component SwitchBlock3 switchBlock3;
                      component Condition3 condition3;
                      component SwitchBlock4 switchBlock4;
                      component Condition4 condition4;
                      component SwitchBlock5 switchBlock5;
                      component Condition5 condition5;
                      component SwitchBlock6 switchBlock6;
                      component Condition6 condition6;
                      component SwitchBlock7 switchBlock7;
                      component Condition7 condition7;
                      component SwitchBlock8 switchBlock8;
                      component Condition8 condition8;
                      component SwitchBlock9 switchBlock9;
                      component Condition9 condition9;
                      component SwitchBlock10 switchBlock10;
                      component Condition10 condition10;
                      component SwitchBlock11 switchBlock11;
                      component Condition11 condition11;
                      component SwitchBlock12 switchBlock12;
                      component Condition12 condition12;
                      component SwitchBlock13 switchBlock13;
                      component Condition13 condition13;
                      component SwitchBlock14 switchBlock14;
                      component Condition14 condition14;
                      component SwitchBlock15 switchBlock15;
                      component Condition15 condition15;
                      component SwitchBlock16 switchBlock16;
                      component Condition16 condition16;
                      component SwitchBlock17 switchBlock17;
                      component Condition17 condition17;
                      component SwitchBlock18 switchBlock18;
                      component Condition18 condition18;
                      component SwitchBlock19 switchBlock19;
                      component Condition19 condition19;
                      component SwitchBlock20 switchBlock20;
                      component Condition20 condition20;
                      component SwitchBlock21 switchBlock21;
                      component Condition21 condition21;
                      component SwitchBlock22 switchBlock22;
                      component Condition22 condition22;
                      component SwitchBlock23 switchBlock23;
                      component Condition23 condition23;
                      component SwitchBlock24 switchBlock24;
                      component Condition24 condition24;
                      component SwitchBlock25 switchBlock25;
                      component Condition25 condition25;
                      component SwitchBlock26 switchBlock26;
                      component Condition26 condition26;
                      component SwitchBlock27 switchBlock27;
                      component Condition27 condition27;
                      component SwitchBlock28 switchBlock28;
                      component Condition28 condition28;
                      component SwitchBlock29 switchBlock29;
                      component Condition29 condition29;
                      component SwitchBlock30 switchBlock30;
                      component Condition30 condition30;
                      component SwitchBlock31 switchBlock31;
                      component Condition31 condition31;
                      component SwitchBlock32 switchBlock32;
                      component Condition32 condition32;
                      component SwitchBlock33 switchBlock33;
                      component Condition33 condition33;
                      component SwitchBlock34 switchBlock34;
                      component Condition34 condition34;
                      component SwitchBlock35 switchBlock35;
                      component Condition35 condition35;
                      component SwitchBlock36 switchBlock36;
                      component Condition36 condition36;
                      component SwitchBlock37 switchBlock37;
                      component Condition37 condition37;
                      component SwitchBlock38 switchBlock38;
                      component Condition38 condition38;
                      component SwitchBlock39 switchBlock39;
                      component Condition39 condition39;
                      component SwitchBlock40 switchBlock40;
                      component Condition40 condition40;
                      component SwitchBlock41 switchBlock41;
                      component Condition41 condition41;
                      component SwitchBlock42 switchBlock42;
                      component Condition42 condition42;
                      component SwitchBlock43 switchBlock43;
                      component Condition43 condition43;
                      component SwitchBlock44 switchBlock44;
                      component Condition44 condition44;
                      component SwitchBlock45 switchBlock45;
                      component Condition45 condition45;
                      component SwitchBlock46 switchBlock46;
                      component Condition46 condition46;
                      component SwitchBlock47 switchBlock47;
                      component Condition47 condition47;
                      component SwitchBlock48 switchBlock48;
                      component Condition48 condition48;
                      component SwitchBlock49 switchBlock49;
                      component Condition49 condition49;
                      component SwitchBlock50 switchBlock50;
                      component Condition50 condition50;
                      component SwitchBlock51 switchBlock51;
                      component Condition51 condition51;
                      component SwitchBlock52 switchBlock52;
                      component Condition52 condition52;
                      component SwitchBlock53 switchBlock53;
                      component Condition53 condition53;
                      component SwitchBlock54 switchBlock54;
                      component Condition54 condition54;
                      component SwitchBlock55 switchBlock55;
                      component Condition55 condition55;
                      component SwitchBlock56 switchBlock56;
                      component Condition56 condition56;
                      component SwitchBlock57 switchBlock57;
                      component Condition57 condition57;
                      component SwitchBlock58 switchBlock58;
                      component Condition58 condition58;
                      component SwitchBlock59 switchBlock59;
                      component Condition59 condition59;
                      component SwitchBlock60 switchBlock60;
                      component Condition60 condition60;
                      component SwitchBlock61 switchBlock61;
                      component Condition61 condition61;
                      component SwitchBlock62 switchBlock62;
                      component Condition62 condition62;
                      component SwitchBlock63 switchBlock63;
                      component Condition63 condition63;
                      component SwitchBlock64 switchBlock64;
                      component Condition64 condition64;
                      component SwitchBlock65 switchBlock65;
                      component Condition65 condition65;
                      component SwitchBlock66 switchBlock66;
                      component Condition66 condition66;
                      component SwitchBlock67 switchBlock67;
                      component Condition67 condition67;
                      component SwitchBlock68 switchBlock68;
                      component Condition68 condition68;
                      component SwitchBlock69 switchBlock69;
                      component Condition69 condition69;
                      component SwitchBlock70 switchBlock70;
                      component Condition70 condition70;
                      component SwitchBlock71 switchBlock71;
                      component Condition71 condition71;
                      connect in1 -> v_SetValuePlusLvl2.v_CCSetValueIn1;
                      connect in2 -> v_SetValuePlus.v_CCSetValueIn1;
                      connect in3 -> v_SetValueMinus.v_CCSetValueIn1;
                      connect in4 -> v_SetValueMinusLvl2.v_CCSetValueIn1;
                      connect relOp2.out1 -> risingEdgeDetector.in1In1;
                      connect relOp3.out1 -> risingEdgeDetector1.in1In1;
                      connect relOp4.out1 -> risingEdgeDetector2.in1In1;
                      connect relOp1.out1 -> risingEdgeDetector3.in1In1;
                      connect risingEdgeDetector1.out1Out1 -> condition.in1;
                      connect v_SetValuePlusLvl2.out1 -> switchBlock.ifIn;
                      connect condition.out1 -> switchBlock.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition1.in1;
                      connect v_SetValuePlus.out1 -> switchBlock1.ifIn;
                      connect condition1.out1 -> switchBlock1.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition2.in1;
                      connect v_SetValueMinusLvl2.out1 -> switchBlock2.ifIn;
                      connect condition2.out1 -> switchBlock2.condition;
                      connect risingEdgeDetector.out1Out1 -> condition3.in1;
                      connect v_SetValueMinus.out1 -> switchBlock3.ifIn;
                      connect condition3.out1 -> switchBlock3.condition;
                      connect in5 -> switchBlock3.elseIn;
                      connect switchBlock3.out1 -> switchBlock2.elseIn;
                      connect switchBlock2.out1 -> switchBlock1.elseIn;
                      connect switchBlock1.out1 -> switchBlock.elseIn;
                      connect switchBlock.out1 -> out1;
                      connect risingEdgeDetector1.out1Out1 -> condition4.in1;
                      connect v_SetValuePlusLvl2.out2 -> switchBlock4.ifIn;
                      connect condition4.out1 -> switchBlock4.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition5.in1;
                      connect v_SetValuePlus.out2 -> switchBlock5.ifIn;
                      connect condition5.out1 -> switchBlock5.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition6.in1;
                      connect v_SetValueMinusLvl2.out2 -> switchBlock6.ifIn;
                      connect condition6.out1 -> switchBlock6.condition;
                      connect risingEdgeDetector.out1Out1 -> condition7.in1;
                      connect v_SetValueMinus.out2 -> switchBlock7.ifIn;
                      connect condition7.out1 -> switchBlock7.condition;
                      connect in6 -> switchBlock7.elseIn;
                      connect switchBlock7.out1 -> switchBlock6.elseIn;
                      connect switchBlock6.out1 -> switchBlock5.elseIn;
                      connect switchBlock5.out1 -> switchBlock4.elseIn;
                      connect switchBlock4.out1 -> out2;
                      connect risingEdgeDetector1.out1Out1 -> condition8.in1;
                      connect v_SetValuePlusLvl2.out3 -> switchBlock8.ifIn;
                      connect condition8.out1 -> switchBlock8.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition9.in1;
                      connect v_SetValuePlus.out3 -> switchBlock9.ifIn;
                      connect condition9.out1 -> switchBlock9.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition10.in1;
                      connect v_SetValueMinusLvl2.out3 -> switchBlock10.ifIn;
                      connect condition10.out1 -> switchBlock10.condition;
                      connect risingEdgeDetector.out1Out1 -> condition11.in1;
                      connect v_SetValueMinus.out3 -> switchBlock11.ifIn;
                      connect condition11.out1 -> switchBlock11.condition;
                      connect in7 -> switchBlock11.elseIn;
                      connect switchBlock11.out1 -> switchBlock10.elseIn;
                      connect switchBlock10.out1 -> switchBlock9.elseIn;
                      connect switchBlock9.out1 -> switchBlock8.elseIn;
                      connect switchBlock8.out1 -> out3;
                      connect risingEdgeDetector1.out1Out1 -> condition12.in1;
                      connect v_SetValuePlusLvl2.out4 -> switchBlock12.ifIn;
                      connect condition12.out1 -> switchBlock12.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition13.in1;
                      connect v_SetValuePlus.out4 -> switchBlock13.ifIn;
                      connect condition13.out1 -> switchBlock13.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition14.in1;
                      connect v_SetValueMinusLvl2.out4 -> switchBlock14.ifIn;
                      connect condition14.out1 -> switchBlock14.condition;
                      connect risingEdgeDetector.out1Out1 -> condition15.in1;
                      connect v_SetValueMinus.out4 -> switchBlock15.ifIn;
                      connect condition15.out1 -> switchBlock15.condition;
                      connect in8 -> switchBlock15.elseIn;
                      connect switchBlock15.out1 -> switchBlock14.elseIn;
                      connect switchBlock14.out1 -> switchBlock13.elseIn;
                      connect switchBlock13.out1 -> switchBlock12.elseIn;
                      connect switchBlock12.out1 -> out4;
                      connect risingEdgeDetector1.out1Out1 -> condition16.in1;
                      connect v_SetValuePlusLvl2.out5 -> switchBlock16.ifIn;
                      connect condition16.out1 -> switchBlock16.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition17.in1;
                      connect v_SetValuePlus.out5 -> switchBlock17.ifIn;
                      connect condition17.out1 -> switchBlock17.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition18.in1;
                      connect v_SetValueMinusLvl2.out5 -> switchBlock18.ifIn;
                      connect condition18.out1 -> switchBlock18.condition;
                      connect risingEdgeDetector.out1Out1 -> condition19.in1;
                      connect v_SetValueMinus.out5 -> switchBlock19.ifIn;
                      connect condition19.out1 -> switchBlock19.condition;
                      connect in9 -> switchBlock19.elseIn;
                      connect switchBlock19.out1 -> switchBlock18.elseIn;
                      connect switchBlock18.out1 -> switchBlock17.elseIn;
                      connect switchBlock17.out1 -> switchBlock16.elseIn;
                      connect switchBlock16.out1 -> out5;
                      connect risingEdgeDetector1.out1Out1 -> condition20.in1;
                      connect v_SetValuePlusLvl2.out6 -> switchBlock20.ifIn;
                      connect condition20.out1 -> switchBlock20.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition21.in1;
                      connect v_SetValuePlus.out6 -> switchBlock21.ifIn;
                      connect condition21.out1 -> switchBlock21.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition22.in1;
                      connect v_SetValueMinusLvl2.out6 -> switchBlock22.ifIn;
                      connect condition22.out1 -> switchBlock22.condition;
                      connect risingEdgeDetector.out1Out1 -> condition23.in1;
                      connect v_SetValueMinus.out6 -> switchBlock23.ifIn;
                      connect condition23.out1 -> switchBlock23.condition;
                      connect in10 -> switchBlock23.elseIn;
                      connect switchBlock23.out1 -> switchBlock22.elseIn;
                      connect switchBlock22.out1 -> switchBlock21.elseIn;
                      connect switchBlock21.out1 -> switchBlock20.elseIn;
                      connect switchBlock20.out1 -> out6;
                      connect risingEdgeDetector1.out1Out1 -> condition24.in1;
                      connect v_SetValuePlusLvl2.out7 -> switchBlock24.ifIn;
                      connect condition24.out1 -> switchBlock24.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition25.in1;
                      connect v_SetValuePlus.out7 -> switchBlock25.ifIn;
                      connect condition25.out1 -> switchBlock25.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition26.in1;
                      connect v_SetValueMinusLvl2.out7 -> switchBlock26.ifIn;
                      connect condition26.out1 -> switchBlock26.condition;
                      connect risingEdgeDetector.out1Out1 -> condition27.in1;
                      connect v_SetValueMinus.out7 -> switchBlock27.ifIn;
                      connect condition27.out1 -> switchBlock27.condition;
                      connect in11 -> switchBlock27.elseIn;
                      connect switchBlock27.out1 -> switchBlock26.elseIn;
                      connect switchBlock26.out1 -> switchBlock25.elseIn;
                      connect switchBlock25.out1 -> switchBlock24.elseIn;
                      connect switchBlock24.out1 -> out7;
                      connect risingEdgeDetector1.out1Out1 -> condition28.in1;
                      connect v_SetValuePlusLvl2.out8 -> switchBlock28.ifIn;
                      connect condition28.out1 -> switchBlock28.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition29.in1;
                      connect v_SetValuePlus.out8 -> switchBlock29.ifIn;
                      connect condition29.out1 -> switchBlock29.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition30.in1;
                      connect v_SetValueMinusLvl2.out8 -> switchBlock30.ifIn;
                      connect condition30.out1 -> switchBlock30.condition;
                      connect risingEdgeDetector.out1Out1 -> condition31.in1;
                      connect v_SetValueMinus.out8 -> switchBlock31.ifIn;
                      connect condition31.out1 -> switchBlock31.condition;
                      connect in12 -> switchBlock31.elseIn;
                      connect switchBlock31.out1 -> switchBlock30.elseIn;
                      connect switchBlock30.out1 -> switchBlock29.elseIn;
                      connect switchBlock29.out1 -> switchBlock28.elseIn;
                      connect switchBlock28.out1 -> out8;
                      connect risingEdgeDetector1.out1Out1 -> condition32.in1;
                      connect v_SetValuePlusLvl2.out9 -> switchBlock32.ifIn;
                      connect condition32.out1 -> switchBlock32.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition33.in1;
                      connect v_SetValuePlus.out9 -> switchBlock33.ifIn;
                      connect condition33.out1 -> switchBlock33.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition34.in1;
                      connect v_SetValueMinusLvl2.out9 -> switchBlock34.ifIn;
                      connect condition34.out1 -> switchBlock34.condition;
                      connect risingEdgeDetector.out1Out1 -> condition35.in1;
                      connect v_SetValueMinus.out9 -> switchBlock35.ifIn;
                      connect condition35.out1 -> switchBlock35.condition;
                      connect in13 -> switchBlock35.elseIn;
                      connect switchBlock35.out1 -> switchBlock34.elseIn;
                      connect switchBlock34.out1 -> switchBlock33.elseIn;
                      connect switchBlock33.out1 -> switchBlock32.elseIn;
                      connect switchBlock32.out1 -> out9;
                      connect risingEdgeDetector1.out1Out1 -> condition36.in1;
                      connect v_SetValuePlusLvl2.out10 -> switchBlock36.ifIn;
                      connect condition36.out1 -> switchBlock36.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition37.in1;
                      connect v_SetValuePlus.out10 -> switchBlock37.ifIn;
                      connect condition37.out1 -> switchBlock37.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition38.in1;
                      connect v_SetValueMinusLvl2.out10 -> switchBlock38.ifIn;
                      connect condition38.out1 -> switchBlock38.condition;
                      connect risingEdgeDetector.out1Out1 -> condition39.in1;
                      connect v_SetValueMinus.out10 -> switchBlock39.ifIn;
                      connect condition39.out1 -> switchBlock39.condition;
                      connect in14 -> switchBlock39.elseIn;
                      connect switchBlock39.out1 -> switchBlock38.elseIn;
                      connect switchBlock38.out1 -> switchBlock37.elseIn;
                      connect switchBlock37.out1 -> switchBlock36.elseIn;
                      connect switchBlock36.out1 -> out10;
                      connect risingEdgeDetector1.out1Out1 -> condition40.in1;
                      connect v_SetValuePlusLvl2.out11 -> switchBlock40.ifIn;
                      connect condition40.out1 -> switchBlock40.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition41.in1;
                      connect v_SetValuePlus.out11 -> switchBlock41.ifIn;
                      connect condition41.out1 -> switchBlock41.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition42.in1;
                      connect v_SetValueMinusLvl2.out11 -> switchBlock42.ifIn;
                      connect condition42.out1 -> switchBlock42.condition;
                      connect risingEdgeDetector.out1Out1 -> condition43.in1;
                      connect v_SetValueMinus.out11 -> switchBlock43.ifIn;
                      connect condition43.out1 -> switchBlock43.condition;
                      connect in15 -> switchBlock43.elseIn;
                      connect switchBlock43.out1 -> switchBlock42.elseIn;
                      connect switchBlock42.out1 -> switchBlock41.elseIn;
                      connect switchBlock41.out1 -> switchBlock40.elseIn;
                      connect switchBlock40.out1 -> out11;
                      connect risingEdgeDetector1.out1Out1 -> condition44.in1;
                      connect v_SetValuePlusLvl2.out12 -> switchBlock44.ifIn;
                      connect condition44.out1 -> switchBlock44.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition45.in1;
                      connect v_SetValuePlus.out12 -> switchBlock45.ifIn;
                      connect condition45.out1 -> switchBlock45.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition46.in1;
                      connect v_SetValueMinusLvl2.out12 -> switchBlock46.ifIn;
                      connect condition46.out1 -> switchBlock46.condition;
                      connect risingEdgeDetector.out1Out1 -> condition47.in1;
                      connect v_SetValueMinus.out12 -> switchBlock47.ifIn;
                      connect condition47.out1 -> switchBlock47.condition;
                      connect in16 -> switchBlock47.elseIn;
                      connect switchBlock47.out1 -> switchBlock46.elseIn;
                      connect switchBlock46.out1 -> switchBlock45.elseIn;
                      connect switchBlock45.out1 -> switchBlock44.elseIn;
                      connect switchBlock44.out1 -> out12;
                      connect risingEdgeDetector1.out1Out1 -> condition48.in1;
                      connect v_SetValuePlusLvl2.out13 -> switchBlock48.ifIn;
                      connect condition48.out1 -> switchBlock48.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition49.in1;
                      connect v_SetValuePlus.out13 -> switchBlock49.ifIn;
                      connect condition49.out1 -> switchBlock49.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition50.in1;
                      connect v_SetValueMinusLvl2.out13 -> switchBlock50.ifIn;
                      connect condition50.out1 -> switchBlock50.condition;
                      connect risingEdgeDetector.out1Out1 -> condition51.in1;
                      connect v_SetValueMinus.out13 -> switchBlock51.ifIn;
                      connect condition51.out1 -> switchBlock51.condition;
                      connect in17 -> switchBlock51.elseIn;
                      connect switchBlock51.out1 -> switchBlock50.elseIn;
                      connect switchBlock50.out1 -> switchBlock49.elseIn;
                      connect switchBlock49.out1 -> switchBlock48.elseIn;
                      connect switchBlock48.out1 -> out13;
                      connect risingEdgeDetector1.out1Out1 -> condition52.in1;
                      connect v_SetValuePlusLvl2.out14 -> switchBlock52.ifIn;
                      connect condition52.out1 -> switchBlock52.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition53.in1;
                      connect v_SetValuePlus.out14 -> switchBlock53.ifIn;
                      connect condition53.out1 -> switchBlock53.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition54.in1;
                      connect v_SetValueMinusLvl2.out14 -> switchBlock54.ifIn;
                      connect condition54.out1 -> switchBlock54.condition;
                      connect risingEdgeDetector.out1Out1 -> condition55.in1;
                      connect v_SetValueMinus.out14 -> switchBlock55.ifIn;
                      connect condition55.out1 -> switchBlock55.condition;
                      connect in18 -> switchBlock55.elseIn;
                      connect switchBlock55.out1 -> switchBlock54.elseIn;
                      connect switchBlock54.out1 -> switchBlock53.elseIn;
                      connect switchBlock53.out1 -> switchBlock52.elseIn;
                      connect switchBlock52.out1 -> out14;
                      connect risingEdgeDetector1.out1Out1 -> condition56.in1;
                      connect v_SetValuePlusLvl2.out15 -> switchBlock56.ifIn;
                      connect condition56.out1 -> switchBlock56.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition57.in1;
                      connect v_SetValuePlus.out15 -> switchBlock57.ifIn;
                      connect condition57.out1 -> switchBlock57.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition58.in1;
                      connect v_SetValueMinusLvl2.out15 -> switchBlock58.ifIn;
                      connect condition58.out1 -> switchBlock58.condition;
                      connect risingEdgeDetector.out1Out1 -> condition59.in1;
                      connect v_SetValueMinus.out15 -> switchBlock59.ifIn;
                      connect condition59.out1 -> switchBlock59.condition;
                      connect in19 -> switchBlock59.elseIn;
                      connect switchBlock59.out1 -> switchBlock58.elseIn;
                      connect switchBlock58.out1 -> switchBlock57.elseIn;
                      connect switchBlock57.out1 -> switchBlock56.elseIn;
                      connect switchBlock56.out1 -> out15;
                      connect risingEdgeDetector1.out1Out1 -> condition60.in1;
                      connect v_SetValuePlusLvl2.out16 -> switchBlock60.ifIn;
                      connect condition60.out1 -> switchBlock60.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition61.in1;
                      connect v_SetValuePlus.out16 -> switchBlock61.ifIn;
                      connect condition61.out1 -> switchBlock61.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition62.in1;
                      connect v_SetValueMinusLvl2.out16 -> switchBlock62.ifIn;
                      connect condition62.out1 -> switchBlock62.condition;
                      connect risingEdgeDetector.out1Out1 -> condition63.in1;
                      connect v_SetValueMinus.out16 -> switchBlock63.ifIn;
                      connect condition63.out1 -> switchBlock63.condition;
                      connect in20 -> switchBlock63.elseIn;
                      connect switchBlock63.out1 -> switchBlock62.elseIn;
                      connect switchBlock62.out1 -> switchBlock61.elseIn;
                      connect switchBlock61.out1 -> switchBlock60.elseIn;
                      connect switchBlock60.out1 -> out16;
                      connect risingEdgeDetector1.out1Out1 -> condition64.in1;
                      connect v_SetValuePlusLvl2.out17 -> switchBlock64.ifIn;
                      connect condition64.out1 -> switchBlock64.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition65.in1;
                      connect v_SetValuePlus.out17 -> switchBlock65.ifIn;
                      connect condition65.out1 -> switchBlock65.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition66.in1;
                      connect v_SetValueMinusLvl2.out17 -> switchBlock66.ifIn;
                      connect condition66.out1 -> switchBlock66.condition;
                      connect risingEdgeDetector.out1Out1 -> condition67.in1;
                      connect v_SetValueMinus.out17 -> switchBlock67.ifIn;
                      connect condition67.out1 -> switchBlock67.condition;
                      connect in21 -> switchBlock67.elseIn;
                      connect switchBlock67.out1 -> switchBlock66.elseIn;
                      connect switchBlock66.out1 -> switchBlock65.elseIn;
                      connect switchBlock65.out1 -> switchBlock64.elseIn;
                      connect switchBlock64.out1 -> out17;
                      connect risingEdgeDetector1.out1Out1 -> condition68.in1;
                      connect v_SetValuePlusLvl2.out18 -> switchBlock68.ifIn;
                      connect condition68.out1 -> switchBlock68.condition;
                      connect risingEdgeDetector3.out1Out1 -> condition69.in1;
                      connect v_SetValuePlus.out18 -> switchBlock69.ifIn;
                      connect condition69.out1 -> switchBlock69.condition;
                      connect risingEdgeDetector2.out1Out1 -> condition70.in1;
                      connect v_SetValueMinusLvl2.out18 -> switchBlock70.ifIn;
                      connect condition70.out1 -> switchBlock70.condition;
                      connect risingEdgeDetector.out1Out1 -> condition71.in1;
                      connect v_SetValueMinus.out18 -> switchBlock71.ifIn;
                      connect condition71.out1 -> switchBlock71.condition;
                      connect in22 -> switchBlock71.elseIn;
                      connect switchBlock71.out1 -> switchBlock70.elseIn;
                      connect switchBlock70.out1 -> switchBlock69.elseIn;
                      connect switchBlock69.out1 -> switchBlock68.elseIn;
                      connect switchBlock68.out1 -> out18;
                      connect constant4.out1 -> relOp4.in2;
                      connect leverDown_statIn2 -> relOp2.in1;
                      connect leverDown_statIn2 -> relOp4.in1;
                      connect constant2.out1 -> relOp2.in2;
                      connect constant1.out1 -> relOp1.in2;
                      connect constant3.out1 -> relOp3.in2;
                      connect leverUp_statIn1 -> relOp3.in1;
                      connect leverUp_statIn1 -> relOp1.in1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    <<Type="Switch">> component SwitchBlock {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock1 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition1 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock2 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition2 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock3 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition3 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock4 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition4 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock5 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition5 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock6 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition6 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock7 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition7 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock8 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition8 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock9 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition9 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock10 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition10 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock11 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition11 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock12 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition12 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock13 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition13 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock14 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition14 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock15 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition15 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock16 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition16 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock17 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition17 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock18 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition18 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock19 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition19 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock20 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition20 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock21 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition21 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock22 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition22 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock23 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition23 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock24 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="~u1",Type="Condition">> component Condition24 {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CC_ChangeSetValue_Lvl2_Repeater cC_ChangeSetValue_Lvl2_Repeater;
                    component CC_ChangeSetValue_Lvl2_no_Repeater cC_ChangeSetValue_Lvl2_no_Repeater;
                    component Constant constant;
                    component VERSION_INFO vERSION_INFO;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    component SwitchBlock1 switchBlock1;
                    component Condition1 condition1;
                    component SwitchBlock2 switchBlock2;
                    component Condition2 condition2;
                    component SwitchBlock3 switchBlock3;
                    component Condition3 condition3;
                    component SwitchBlock4 switchBlock4;
                    component Condition4 condition4;
                    component SwitchBlock5 switchBlock5;
                    component Condition5 condition5;
                    component SwitchBlock6 switchBlock6;
                    component Condition6 condition6;
                    component SwitchBlock7 switchBlock7;
                    component Condition7 condition7;
                    component SwitchBlock8 switchBlock8;
                    component Condition8 condition8;
                    component SwitchBlock9 switchBlock9;
                    component Condition9 condition9;
                    component SwitchBlock10 switchBlock10;
                    component Condition10 condition10;
                    component SwitchBlock11 switchBlock11;
                    component Condition11 condition11;
                    component SwitchBlock12 switchBlock12;
                    component Condition12 condition12;
                    component SwitchBlock13 switchBlock13;
                    component Condition13 condition13;
                    component SwitchBlock14 switchBlock14;
                    component Condition14 condition14;
                    component SwitchBlock15 switchBlock15;
                    component Condition15 condition15;
                    component SwitchBlock16 switchBlock16;
                    component Condition16 condition16;
                    component SwitchBlock17 switchBlock17;
                    component Condition17 condition17;
                    component SwitchBlock18 switchBlock18;
                    component Condition18 condition18;
                    component SwitchBlock19 switchBlock19;
                    component Condition19 condition19;
                    component SwitchBlock20 switchBlock20;
                    component Condition20 condition20;
                    component SwitchBlock21 switchBlock21;
                    component Condition21 condition21;
                    component SwitchBlock22 switchBlock22;
                    component Condition22 condition22;
                    component SwitchBlock23 switchBlock23;
                    component Condition23 condition23;
                    component SwitchBlock24 switchBlock24;
                    component Condition24 condition24;
                    connect in1 -> cC_ChangeSetValue_Lvl2_no_Repeater.in1;
                    connect in2 -> cC_ChangeSetValue_Lvl2_no_Repeater.in2;
                    connect in3 -> cC_ChangeSetValue_Lvl2_no_Repeater.in3;
                    connect in4 -> cC_ChangeSetValue_Lvl2_no_Repeater.in4;
                    connect in5 -> cC_ChangeSetValue_Lvl2_no_Repeater.in5;
                    connect constant.out1 -> condition.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out1 -> switchBlock.ifIn;
                    connect condition.out1 -> switchBlock.condition;
                    connect in6 -> switchBlock.elseIn;
                    connect switchBlock.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in1;
                    connect in7 -> cC_ChangeSetValue_Lvl2_no_Repeater.in6;
                    connect constant.out1 -> condition1.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out2 -> switchBlock1.ifIn;
                    connect condition1.out1 -> switchBlock1.condition;
                    connect in8 -> switchBlock1.elseIn;
                    connect switchBlock1.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in2;
                    connect in9 -> cC_ChangeSetValue_Lvl2_no_Repeater.in7;
                    connect constant.out1 -> condition2.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out3 -> switchBlock2.ifIn;
                    connect condition2.out1 -> switchBlock2.condition;
                    connect in10 -> switchBlock2.elseIn;
                    connect switchBlock2.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in3;
                    connect in11 -> cC_ChangeSetValue_Lvl2_no_Repeater.in8;
                    connect constant.out1 -> condition3.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out4 -> switchBlock3.ifIn;
                    connect condition3.out1 -> switchBlock3.condition;
                    connect in12 -> switchBlock3.elseIn;
                    connect switchBlock3.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in4;
                    connect in13 -> cC_ChangeSetValue_Lvl2_no_Repeater.in9;
                    connect constant.out1 -> condition4.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out5 -> switchBlock4.ifIn;
                    connect condition4.out1 -> switchBlock4.condition;
                    connect in14 -> switchBlock4.elseIn;
                    connect switchBlock4.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in5;
                    connect constant.out1 -> condition5.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out1 -> switchBlock5.ifIn;
                    connect condition5.out1 -> switchBlock5.condition;
                    connect in15 -> cC_ChangeSetValue_Lvl2_no_Repeater.in10;
                    connect constant.out1 -> condition6.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out6 -> switchBlock6.ifIn;
                    connect condition6.out1 -> switchBlock6.condition;
                    connect in16 -> switchBlock6.elseIn;
                    connect switchBlock6.out1 -> switchBlock5.elseIn;
                    connect switchBlock5.out1 -> out1;
                    connect in17 -> cC_ChangeSetValue_Lvl2_no_Repeater.in11;
                    connect constant.out1 -> condition7.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out7 -> switchBlock7.ifIn;
                    connect condition7.out1 -> switchBlock7.condition;
                    connect in18 -> switchBlock7.elseIn;
                    connect switchBlock7.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in6;
                    connect constant.out1 -> condition8.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out2 -> switchBlock8.ifIn;
                    connect condition8.out1 -> switchBlock8.condition;
                    connect in19 -> cC_ChangeSetValue_Lvl2_no_Repeater.in12;
                    connect constant.out1 -> condition9.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out8 -> switchBlock9.ifIn;
                    connect condition9.out1 -> switchBlock9.condition;
                    connect in20 -> switchBlock9.elseIn;
                    connect switchBlock9.out1 -> switchBlock8.elseIn;
                    connect switchBlock8.out1 -> out2;
                    connect in21 -> cC_ChangeSetValue_Lvl2_no_Repeater.in13;
                    connect constant.out1 -> condition10.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out9 -> switchBlock10.ifIn;
                    connect condition10.out1 -> switchBlock10.condition;
                    connect in22 -> switchBlock10.elseIn;
                    connect switchBlock10.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in7;
                    connect constant.out1 -> condition11.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out3 -> switchBlock11.ifIn;
                    connect condition11.out1 -> switchBlock11.condition;
                    connect in23 -> cC_ChangeSetValue_Lvl2_no_Repeater.in14;
                    connect constant.out1 -> condition12.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out10 -> switchBlock12.ifIn;
                    connect condition12.out1 -> switchBlock12.condition;
                    connect in24 -> switchBlock12.elseIn;
                    connect switchBlock12.out1 -> switchBlock11.elseIn;
                    connect switchBlock11.out1 -> out3;
                    connect in25 -> cC_ChangeSetValue_Lvl2_no_Repeater.in15;
                    connect constant.out1 -> condition13.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out11 -> switchBlock13.ifIn;
                    connect condition13.out1 -> switchBlock13.condition;
                    connect in26 -> switchBlock13.elseIn;
                    connect switchBlock13.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in8;
                    connect constant.out1 -> condition14.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out4 -> switchBlock14.ifIn;
                    connect condition14.out1 -> switchBlock14.condition;
                    connect in27 -> cC_ChangeSetValue_Lvl2_no_Repeater.in16;
                    connect constant.out1 -> condition15.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out12 -> switchBlock15.ifIn;
                    connect condition15.out1 -> switchBlock15.condition;
                    connect in28 -> switchBlock15.elseIn;
                    connect switchBlock15.out1 -> switchBlock14.elseIn;
                    connect switchBlock14.out1 -> out4;
                    connect in29 -> cC_ChangeSetValue_Lvl2_no_Repeater.in17;
                    connect constant.out1 -> condition16.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out13 -> switchBlock16.ifIn;
                    connect condition16.out1 -> switchBlock16.condition;
                    connect in30 -> switchBlock16.elseIn;
                    connect switchBlock16.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in9;
                    connect constant.out1 -> condition17.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out5 -> switchBlock17.ifIn;
                    connect condition17.out1 -> switchBlock17.condition;
                    connect in31 -> cC_ChangeSetValue_Lvl2_no_Repeater.in18;
                    connect constant.out1 -> condition18.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out14 -> switchBlock18.ifIn;
                    connect condition18.out1 -> switchBlock18.condition;
                    connect in32 -> switchBlock18.elseIn;
                    connect switchBlock18.out1 -> switchBlock17.elseIn;
                    connect switchBlock17.out1 -> out5;
                    connect in33 -> cC_ChangeSetValue_Lvl2_no_Repeater.in19;
                    connect constant.out1 -> condition19.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out15 -> switchBlock19.ifIn;
                    connect condition19.out1 -> switchBlock19.condition;
                    connect in34 -> switchBlock19.elseIn;
                    connect switchBlock19.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in10;
                    connect constant.out1 -> condition20.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out6 -> switchBlock20.ifIn;
                    connect condition20.out1 -> switchBlock20.condition;
                    connect in35 -> cC_ChangeSetValue_Lvl2_no_Repeater.in20;
                    connect constant.out1 -> condition21.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out16 -> switchBlock21.ifIn;
                    connect condition21.out1 -> switchBlock21.condition;
                    connect in36 -> switchBlock21.elseIn;
                    connect switchBlock21.out1 -> switchBlock20.elseIn;
                    connect switchBlock20.out1 -> out6;
                    connect in37 -> cC_ChangeSetValue_Lvl2_no_Repeater.in21;
                    connect constant.out1 -> condition22.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out17 -> switchBlock22.ifIn;
                    connect condition22.out1 -> switchBlock22.condition;
                    connect in38 -> switchBlock22.elseIn;
                    connect switchBlock22.out1 -> cC_ChangeSetValue_Lvl2_Repeater.in11;
                    connect constant.out1 -> condition23.in1;
                    connect cC_ChangeSetValue_Lvl2_Repeater.out7 -> switchBlock23.ifIn;
                    connect condition23.out1 -> switchBlock23.condition;
                    connect in39 -> cC_ChangeSetValue_Lvl2_no_Repeater.in22;
                    connect constant.out1 -> condition24.in1;
                    connect cC_ChangeSetValue_Lvl2_no_Repeater.out18 -> switchBlock24.ifIn;
                    connect condition24.out1 -> switchBlock24.condition;
                    connect in40 -> switchBlock24.elseIn;
                    connect switchBlock24.out1 -> switchBlock23.elseIn;
                    connect switchBlock23.out1 -> out7;
                    connect leverDown_statIn2 -> cC_ChangeSetValue_Lvl2_no_Repeater.leverDown_statIn2;
                    connect leverDown_statIn2 -> cC_ChangeSetValue_Lvl2_Repeater.leverDown_StatIn2;
                    connect leverUp_statIn1 -> cC_ChangeSetValue_Lvl2_no_Repeater.leverUp_statIn1;
                    connect leverUp_statIn1 -> cC_ChangeSetValue_Lvl2_Repeater.leverUp_StatIn1;
                  }
                  <<Type="SubSystem">> component CC_InitialSetValue {
                    ports
                      in Double v_Vehicle_kmhIn1,
                      in Double in1,
                      in Double in2,
                      out Double out1,
                      in Double in3,
                      out Double out2,
                      in Double in4,
                      out Double out3;
                    <<Type="SubSystem">> component CC_NoInitialSetValue {
                      ports
                        in Double v_Vehicle_kmhIn1,
                        out Double out1,
                        out Double out2,
                        out Double out3;
                      <<Type="SubSystem">> component VERSION_INFO {
                        <<Type="SubSystem">> component Copyright {
                        }
                        component Copyright copyright;
                      }
                      component VERSION_INFO vERSION_INFO;
                      connect v_Vehicle_kmhIn1 -> out1;
                      connect v_Vehicle_kmhIn1 -> out2;
                      connect v_Vehicle_kmhIn1 -> out3;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Operator="<",Type="RelationalOperator">> component RelOp {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    <<Type="Switch">> component SwitchBlock {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock1 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition1 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch">> component SwitchBlock2 {
                      ports
                        in Double ifIn,
                        in Boolean condition,
                        in Double elseIn,
                        out Double out1;
                    }
                    <<Condition="u1",Type="Condition">> component Condition2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component CC_NoInitialSetValue cC_NoInitialSetValue;
                    component Constant constant;
                    component RelOp relOp;
                    component VERSION_INFO vERSION_INFO;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    component SwitchBlock1 switchBlock1;
                    component Condition1 condition1;
                    component SwitchBlock2 switchBlock2;
                    component Condition2 condition2;
                    connect in1 -> relOp.in1;
                    connect relOp.out1 -> condition.in1;
                    connect cC_NoInitialSetValue.out1 -> switchBlock.ifIn;
                    connect condition.out1 -> switchBlock.condition;
                    connect in2 -> switchBlock.elseIn;
                    connect switchBlock.out1 -> out1;
                    connect relOp.out1 -> condition1.in1;
                    connect cC_NoInitialSetValue.out2 -> switchBlock1.ifIn;
                    connect condition1.out1 -> switchBlock1.condition;
                    connect in3 -> switchBlock1.elseIn;
                    connect switchBlock1.out1 -> out2;
                    connect relOp.out1 -> condition2.in1;
                    connect cC_NoInitialSetValue.out3 -> switchBlock2.ifIn;
                    connect condition2.out1 -> switchBlock2.condition;
                    connect in4 -> switchBlock2.elseIn;
                    connect switchBlock2.out1 -> out3;
                    connect constant.out1 -> relOp.in2;
                    connect v_Vehicle_kmhIn1 -> cC_NoInitialSetValue.v_Vehicle_kmhIn1;
                  }
                  <<Type="SubSystem">> component CC_OnSet_SetValue {
                    ports
                      in Double v_Vehicle_kmhIn1,
                      out Double out1,
                      out Double out2,
                      out Double out3;
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component VERSION_INFO vERSION_INFO;
                    connect v_Vehicle_kmhIn1 -> out1;
                    connect v_Vehicle_kmhIn1 -> out2;
                    connect v_Vehicle_kmhIn1 -> out3;
                  }
                  <<Type="SubSystem">> component CC_StartUpSetValue {
                    ports
                      out Double out1,
                      out Double out2,
                      out Double out3;
                    <<Type="Constant",Value="-1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="SubSystem">> component VERSION_INFO {
                      <<Type="SubSystem">> component Copyright {
                      }
                      component Copyright copyright;
                    }
                    component Constant constant;
                    component VERSION_INFO vERSION_INFO;
                    connect constant.out1 -> out1;
                    connect constant.out1 -> out2;
                    connect constant.out1 -> out3;
                  }
                  <<Type="SubSystem">> component CompareToZero {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator=">",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="SubSystem">> component CompareToZero1 {
                    ports
                      in Double uIn1,
                      out Boolean yOut1;
                    <<Operator=">",Type="RelationalOperator">> component Compare {
                      ports
                        in Double in1,
                        in Double in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Type="Constant",Value="0">> component Constant {
                      ports
                        out Double out1;
                    }
                    component Compare compare;
                    component Constant constant;
                    connect compare.out1 -> yOut1;
                    connect uIn1 -> compare.in1;
                    connect constant.out1 -> compare.in2;
                  }
                  <<Type="Constant",Value="20">> component Constant {
                    ports
                      out Double out1;
                  }
                  <<Type="SubSystem">> component EdgeRising {
                    ports
                      in Boolean uIn1,
                      in Boolean rIn2,
                      in Boolean iVIn3,
                      out Boolean yOut1;
                    <<Operator="AND",Type="Logic">> component LogOp_A {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                      effect in1 -> out1;
                      effect in2 -> out1;
                    }
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_U {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_A logOp_A;
                    component LogOp_N logOp_N;
                    component Memory_U memory_U;
                    component Switch_R switch_R;
                    component Condition condition;
                    connect condition.out1 -> switch_R.condition;
                    connect rIn2 -> condition.in1;
                    connect logOp_N.out1 -> logOp_A.in2;
                    connect logOp_A.out1 -> yOut1;
                    connect uIn1 -> logOp_A.in1;
                    connect uIn1 -> memory_U.in1;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect memory_U.out1 -> switch_R.elseIn;
                    connect iVIn3 -> switch_R.ifIn;
                  }
                  <<Type="SubSystem">> component FalseBlock {
                    ports
                      out Boolean yOut1;
                    <<Type="Constant",Value="0">> component Zero {
                      ports
                        out Boolean out1;
                    }
                    component Zero zero;
                    connect zero.out1 -> yOut1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator1 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="AND",Type="Logic">> component LogicalOperator2 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      in Boolean in3,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                    effect in3 -> out1;
                  }
                  <<Operator="OR",Type="Logic">> component LogicalOperator3 {
                    ports
                      in Boolean in1,
                      in Boolean in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator4 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator5 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Operator="NOT",Type="Logic">> component LogicalOperator6 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                    effect in1 -> out1;
                  }
                  <<Type="SubSystem">> component RSFlipFlop {
                    ports
                      in Boolean sIn1,
                      in Boolean rIn2,
                      out Boolean qOut1,
                      out Boolean nOT_QOut2;
                    <<Operator="NOT",Type="Logic">> component LogOp_N {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="UnitDelay",InitialCondition="0">> component Memory_Q {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                      effect in1 -> out1;
                    }
                    <<Type="Constant",Value="1">> component One_S {
                      ports
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_S {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                      effect ifIn -> out1;
                      effect condition -> out1;
                      effect elseIn -> out1;
                    }
                    <<Type="Constant",Value="0">> component Zero_R {
                      ports
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    component LogOp_N logOp_N;
                    component Memory_Q memory_Q;
                    component One_S one_S;
                    component Switch_R switch_R;
                    component Switch_S switch_S;
                    component Zero_R zero_R;
                    component Condition condition;
                    component Condition1 condition1;
                    connect condition.out1 -> switch_S.condition;
                    connect condition1.out1 -> switch_R.condition;
                    connect sIn1 -> condition.in1;
                    connect rIn2 -> condition1.in1;
                    connect memory_Q.out1 -> switch_S.elseIn;
                    connect switch_S.out1 -> switch_R.elseIn;
                    connect one_S.out1 -> switch_S.ifIn;
                    connect switch_R.out1 -> memory_Q.in1;
                    connect switch_R.out1 -> qOut1;
                    connect switch_R.out1 -> logOp_N.in1;
                    connect zero_R.out1 -> switch_R.ifIn;
                    connect logOp_N.out1 -> nOT_QOut2;
                  }
                  <<Operator=">=",Type="RelationalOperator">> component RelOp {
                    ports
                      in Double in1,
                      in Double in2,
                      out Boolean out1;
                    effect in1 -> out1;
                    effect in2 -> out1;
                  }
                  <<Type="SubSystem">> component SysInit {
                    ports
                      out Double yOut1;
                    <<Type="UnitDelay",InitialCondition="1">> component Memory_Init {
                      ports
                        in Double in1,
                        out Double out1;
                      effect in1 -> out1;
                    }
                    <<Type="Constant",Value="0">> component Zero_Init {
                      ports
                        out Double out1;
                    }
                    component Memory_Init memory_Init;
                    component Zero_Init zero_Init;
                    connect memory_Init.out1 -> yOut1;
                    connect zero_Init.out1 -> memory_Init.in1;
                  }
                  <<Type="Terminator">> component Terminator {
                    ports
                      in Boolean in1;
                  }
                  <<Type="SubSystem">> component VERSION_INFO {
                    <<Type="SubSystem">> component Copyright {
                    }
                    component Copyright copyright;
                  }
                  <<Type="SubSystem">> component RisingEdgeDetector {
                    ports
                      in Boolean in1In1,
                      out Boolean out1Out1;
                    <<Type="SubSystem">> component CompareToZero {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero1 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero2 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="<",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero3 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">=",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero4 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="Constant",Value="1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="Constant",Value="0">> component Constant1 {
                      ports
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        in Boolean in3,
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory1 {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                    }
                    <<Condition="u2 > 0",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CompareToZero compareToZero;
                    component CompareToZero1 compareToZero1;
                    component CompareToZero2 compareToZero2;
                    component CompareToZero3 compareToZero3;
                    component CompareToZero4 compareToZero4;
                    component Constant constant;
                    component Constant1 constant1;
                    component LogicalOperator logicalOperator;
                    component LogicalOperator1 logicalOperator1;
                    component LogicalOperator2 logicalOperator2;
                    component Memory memory;
                    component Memory1 memory1;
                    component Memory2 memory2;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    connect switchBlock.out1 -> out1Out1;
                    connect compareToZero4.yOut1 -> logicalOperator.in3;
                    connect memory2.out1 -> compareToZero4.uIn1;
                    connect logicalOperator.out1 -> logicalOperator2.in1;
                    connect constant1.out1 -> switchBlock.elseIn;
                    connect constant.out1 -> memory1.in1;
                    connect condition.out1 -> switchBlock.condition;
                    connect logicalOperator2.out1 -> switchBlock.ifIn;
                    connect logicalOperator2.out1 -> memory2.in1;
                    connect logicalOperator1.out1 -> logicalOperator2.in2;
                    connect compareToZero3.yOut1 -> logicalOperator1.in2;
                    connect compareToZero2.yOut1 -> logicalOperator1.in1;
                    connect compareToZero1.yOut1 -> logicalOperator.in2;
                    connect compareToZero.yOut1 -> logicalOperator.in1;
                    connect memory.out1 -> compareToZero.uIn1;
                    connect memory.out1 -> compareToZero2.uIn1;
                    connect in1In1 -> memory.in1;
                    connect in1In1 -> compareToZero1.uIn1;
                    connect in1In1 -> compareToZero3.uIn1;
                    connect memory1.out1 -> condition.in1;
                  }
                  <<Type="SubSystem">> component RisingEdgeDetector1 {
                    ports
                      in Double in1In1,
                      out Boolean out1Out1;
                    <<Type="SubSystem">> component CompareToZero {
                      ports
                        in Double uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero1 {
                      ports
                        in Double uIn1,
                        out Boolean yOut1;
                      <<Operator=">",Type="RelationalOperator">> component Compare {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero2 {
                      ports
                        in Double uIn1,
                        out Boolean yOut1;
                      <<Operator="<",Type="RelationalOperator">> component Compare {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero3 {
                      ports
                        in Double uIn1,
                        out Boolean yOut1;
                      <<Operator=">=",Type="RelationalOperator">> component Compare {
                        ports
                          in Double in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero4 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="Constant",Value="1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="Constant",Value="0">> component Constant1 {
                      ports
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        in Boolean in3,
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory1 {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                    }
                    <<Condition="u2 > 0",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CompareToZero compareToZero;
                    component CompareToZero1 compareToZero1;
                    component CompareToZero2 compareToZero2;
                    component CompareToZero3 compareToZero3;
                    component CompareToZero4 compareToZero4;
                    component Constant constant;
                    component Constant1 constant1;
                    component LogicalOperator logicalOperator;
                    component LogicalOperator1 logicalOperator1;
                    component LogicalOperator2 logicalOperator2;
                    component Memory memory;
                    component Memory1 memory1;
                    component Memory2 memory2;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    connect switchBlock.out1 -> out1Out1;
                    connect compareToZero4.yOut1 -> logicalOperator.in3;
                    connect memory2.out1 -> compareToZero4.uIn1;
                    connect logicalOperator.out1 -> logicalOperator2.in1;
                    connect constant1.out1 -> switchBlock.elseIn;
                    connect constant.out1 -> memory1.in1;
                    connect condition.out1 -> switchBlock.condition;
                    connect logicalOperator2.out1 -> switchBlock.ifIn;
                    connect logicalOperator2.out1 -> memory2.in1;
                    connect logicalOperator1.out1 -> logicalOperator2.in2;
                    connect compareToZero3.yOut1 -> logicalOperator1.in2;
                    connect compareToZero2.yOut1 -> logicalOperator1.in1;
                    connect compareToZero1.yOut1 -> logicalOperator.in2;
                    connect compareToZero.yOut1 -> logicalOperator.in1;
                    connect memory.out1 -> compareToZero.uIn1;
                    connect memory.out1 -> compareToZero2.uIn1;
                    connect in1In1 -> memory.in1;
                    connect in1In1 -> compareToZero1.uIn1;
                    connect in1In1 -> compareToZero3.uIn1;
                    connect memory1.out1 -> condition.in1;
                  }
                  <<Type="SubSystem">> component RisingEdgeDetector2 {
                    ports
                      in Boolean in1In1,
                      out Boolean out1Out1;
                    <<Type="SubSystem">> component CompareToZero {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero1 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero2 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="<",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero3 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator=">=",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="SubSystem">> component CompareToZero4 {
                      ports
                        in Boolean uIn1,
                        out Boolean yOut1;
                      <<Operator="==",Type="RelationalOperator">> component Compare {
                        ports
                          in Boolean in1,
                          in Double in2,
                          out Boolean out1;
                      }
                      <<Type="Constant",Value="0">> component Constant {
                        ports
                          out Double out1;
                      }
                      component Compare compare;
                      component Constant constant;
                      connect compare.out1 -> yOut1;
                      connect uIn1 -> compare.in1;
                      connect constant.out1 -> compare.in2;
                    }
                    <<Type="Constant",Value="1">> component Constant {
                      ports
                        out Double out1;
                    }
                    <<Type="Constant",Value="0">> component Constant1 {
                      ports
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        in Boolean in3,
                        out Boolean out1;
                    }
                    <<Operator="AND",Type="Logic">> component LogicalOperator1 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Operator="OR",Type="Logic">> component LogicalOperator2 {
                      ports
                        in Boolean in1,
                        in Boolean in2,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Memory">> component Memory1 {
                      ports
                        in Double in1,
                        out Double out1;
                    }
                    <<Type="Memory">> component Memory2 {
                      ports
                        in Boolean in1,
                        out Boolean out1;
                    }
                    <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
                      ports
                        in Boolean ifIn,
                        in Boolean condition,
                        in Boolean elseIn,
                        out Boolean out1;
                    }
                    <<Condition="u2 > 0",Type="Condition">> component Condition {
                      ports
                        in Double in1,
                        out Boolean out1;
                    }
                    component CompareToZero compareToZero;
                    component CompareToZero1 compareToZero1;
                    component CompareToZero2 compareToZero2;
                    component CompareToZero3 compareToZero3;
                    component CompareToZero4 compareToZero4;
                    component Constant constant;
                    component Constant1 constant1;
                    component LogicalOperator logicalOperator;
                    component LogicalOperator1 logicalOperator1;
                    component LogicalOperator2 logicalOperator2;
                    component Memory memory;
                    component Memory1 memory1;
                    component Memory2 memory2;
                    component SwitchBlock switchBlock;
                    component Condition condition;
                    connect switchBlock.out1 -> out1Out1;
                    connect compareToZero4.yOut1 -> logicalOperator.in3;
                    connect memory2.out1 -> compareToZero4.uIn1;
                    connect logicalOperator.out1 -> logicalOperator2.in1;
                    connect constant1.out1 -> switchBlock.elseIn;
                    connect constant.out1 -> memory1.in1;
                    connect condition.out1 -> switchBlock.condition;
                    connect logicalOperator2.out1 -> switchBlock.ifIn;
                    connect logicalOperator2.out1 -> memory2.in1;
                    connect logicalOperator1.out1 -> logicalOperator2.in2;
                    connect compareToZero3.yOut1 -> logicalOperator1.in2;
                    connect compareToZero2.yOut1 -> logicalOperator1.in1;
                    connect compareToZero1.yOut1 -> logicalOperator.in2;
                    connect compareToZero.yOut1 -> logicalOperator.in1;
                    connect memory.out1 -> compareToZero.uIn1;
                    connect memory.out1 -> compareToZero2.uIn1;
                    connect in1In1 -> memory.in1;
                    connect in1In1 -> compareToZero1.uIn1;
                    connect in1In1 -> compareToZero3.uIn1;
                    connect memory1.out1 -> condition.in1;
                  }
                  <<Type="Switch">> component SwitchBlock {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock1 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition1 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock2 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition2 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock3 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition3 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock4 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition4 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock5 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition5 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock6 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition6 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock7 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition7 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock8 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition8 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock9 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition9 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock10 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition10 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock11 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition11 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock12 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition12 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock13 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition13 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock14 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition14 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  <<Type="Switch">> component SwitchBlock15 {
                    ports
                      in Double ifIn,
                      in Boolean condition,
                      in Double elseIn,
                      out Double out1;
                  }
                  <<Condition="u1>0",Type="Condition">> component Condition15 {
                    ports
                      in Boolean in1,
                      out Boolean out1;
                  }
                  component CC_ChangeSetValue cC_ChangeSetValue;
                  component CC_InitialSetValue cC_InitialSetValue;
                  component CC_OnSet_SetValue cC_OnSet_SetValue;
                  component CC_StartUpSetValue cC_StartUpSetValue;
                  component CompareToZero compareToZero;
                  component CompareToZero1 compareToZero1;
                  component Constant constant;
                  component EdgeRising edgeRising;
                  component FalseBlock falseBlock;
                  component LogicalOperator logicalOperator;
                  component LogicalOperator1 logicalOperator1;
                  component LogicalOperator2 logicalOperator2;
                  component LogicalOperator3 logicalOperator3;
                  component LogicalOperator4 logicalOperator4;
                  component LogicalOperator5 logicalOperator5;
                  component LogicalOperator6 logicalOperator6;
                  component RSFlipFlop rSFlipFlop;
                  component RelOp relOp;
                  component SysInit sysInit;
                  component Terminator terminator;
                  component VERSION_INFO vERSION_INFO;
                  component RisingEdgeDetector risingEdgeDetector;
                  component RisingEdgeDetector1 risingEdgeDetector1;
                  component RisingEdgeDetector2 risingEdgeDetector2;
                  component SwitchBlock switchBlock;
                  component Condition condition;
                  component SwitchBlock1 switchBlock1;
                  component Condition1 condition1;
                  component SwitchBlock2 switchBlock2;
                  component Condition2 condition2;
                  component SwitchBlock3 switchBlock3;
                  component Condition3 condition3;
                  component SwitchBlock4 switchBlock4;
                  component Condition4 condition4;
                  component SwitchBlock5 switchBlock5;
                  component Condition5 condition5;
                  component SwitchBlock6 switchBlock6;
                  component Condition6 condition6;
                  component SwitchBlock7 switchBlock7;
                  component Condition7 condition7;
                  component SwitchBlock8 switchBlock8;
                  component Condition8 condition8;
                  component SwitchBlock9 switchBlock9;
                  component Condition9 condition9;
                  component SwitchBlock10 switchBlock10;
                  component Condition10 condition10;
                  component SwitchBlock11 switchBlock11;
                  component Condition11 condition11;
                  component SwitchBlock12 switchBlock12;
                  component Condition12 condition12;
                  component SwitchBlock13 switchBlock13;
                  component Condition13 condition13;
                  component SwitchBlock14 switchBlock14;
                  component Condition14 condition14;
                  component SwitchBlock15 switchBlock15;
                  component Condition15 condition15;
                  connect logicalOperator.out1 -> risingEdgeDetector.in1In1;
                  connect sysInit.yOut1 -> risingEdgeDetector1.in1In1;
                  connect logicalOperator2.out1 -> risingEdgeDetector2.in1In1;
                  connect in1 -> cC_ChangeSetValue.in1;
                  connect in2 -> cC_ChangeSetValue.in2;
                  connect in3 -> cC_ChangeSetValue.in3;
                  connect in4 -> cC_ChangeSetValue.in4;
                  connect in5 -> cC_ChangeSetValue.in5;
                  connect in6 -> cC_ChangeSetValue.in6;
                  connect in7 -> cC_ChangeSetValue.in7;
                  connect in8 -> cC_ChangeSetValue.in8;
                  connect in9 -> cC_ChangeSetValue.in9;
                  connect in10 -> cC_ChangeSetValue.in10;
                  connect in11 -> cC_ChangeSetValue.in11;
                  connect in12 -> cC_ChangeSetValue.in12;
                  connect in13 -> cC_ChangeSetValue.in13;
                  connect in14 -> cC_ChangeSetValue.in14;
                  connect in15 -> cC_ChangeSetValue.in15;
                  connect in16 -> cC_ChangeSetValue.in16;
                  connect cC_enabled_bIn2 -> condition.in1;
                  connect cC_ChangeSetValue.out1 -> switchBlock.ifIn;
                  connect condition.out1 -> switchBlock.condition;
                  connect in17 -> switchBlock.elseIn;
                  connect switchBlock.out1 -> cC_InitialSetValue.in1;
                  connect risingEdgeDetector1.out1Out1 -> condition1.in1;
                  connect cC_StartUpSetValue.out1 -> switchBlock1.ifIn;
                  connect condition1.out1 -> switchBlock1.condition;
                  connect risingEdgeDetector2.out1Out1 -> condition2.in1;
                  connect cC_OnSet_SetValue.out1 -> switchBlock2.ifIn;
                  connect condition2.out1 -> switchBlock2.condition;
                  connect in18 -> cC_ChangeSetValue.in17;
                  connect in19 -> cC_ChangeSetValue.in18;
                  connect in20 -> cC_ChangeSetValue.in19;
                  connect in21 -> cC_ChangeSetValue.in20;
                  connect cC_enabled_bIn2 -> condition3.in1;
                  connect cC_ChangeSetValue.out2 -> switchBlock3.ifIn;
                  connect condition3.out1 -> switchBlock3.condition;
                  connect in22 -> switchBlock3.elseIn;
                  connect switchBlock3.out1 -> cC_InitialSetValue.in2;
                  connect risingEdgeDetector.out1Out1 -> condition4.in1;
                  connect cC_InitialSetValue.out1 -> switchBlock4.ifIn;
                  connect condition4.out1 -> switchBlock4.condition;
                  connect in23 -> cC_ChangeSetValue.in21;
                  connect in24 -> cC_ChangeSetValue.in22;
                  connect in25 -> cC_ChangeSetValue.in23;
                  connect in26 -> cC_ChangeSetValue.in24;
                  connect cC_enabled_bIn2 -> condition5.in1;
                  connect cC_ChangeSetValue.out3 -> switchBlock5.ifIn;
                  connect condition5.out1 -> switchBlock5.condition;
                  connect in27 -> switchBlock5.elseIn;
                  connect switchBlock5.out1 -> switchBlock4.elseIn;
                  connect switchBlock4.out1 -> switchBlock2.elseIn;
                  connect switchBlock2.out1 -> switchBlock1.elseIn;
                  connect switchBlock1.out1 -> out1;
                  connect risingEdgeDetector1.out1Out1 -> condition6.in1;
                  connect cC_StartUpSetValue.out2 -> switchBlock6.ifIn;
                  connect condition6.out1 -> switchBlock6.condition;
                  connect risingEdgeDetector2.out1Out1 -> condition7.in1;
                  connect cC_OnSet_SetValue.out2 -> switchBlock7.ifIn;
                  connect condition7.out1 -> switchBlock7.condition;
                  connect in28 -> cC_ChangeSetValue.in25;
                  connect in29 -> cC_ChangeSetValue.in26;
                  connect in30 -> cC_ChangeSetValue.in27;
                  connect in31 -> cC_ChangeSetValue.in28;
                  connect cC_enabled_bIn2 -> condition8.in1;
                  connect cC_ChangeSetValue.out4 -> switchBlock8.ifIn;
                  connect condition8.out1 -> switchBlock8.condition;
                  connect in32 -> switchBlock8.elseIn;
                  connect switchBlock8.out1 -> cC_InitialSetValue.in3;
                  connect risingEdgeDetector.out1Out1 -> condition9.in1;
                  connect cC_InitialSetValue.out2 -> switchBlock9.ifIn;
                  connect condition9.out1 -> switchBlock9.condition;
                  connect in33 -> cC_ChangeSetValue.in29;
                  connect in34 -> cC_ChangeSetValue.in30;
                  connect in35 -> cC_ChangeSetValue.in31;
                  connect in36 -> cC_ChangeSetValue.in32;
                  connect cC_enabled_bIn2 -> condition10.in1;
                  connect cC_ChangeSetValue.out5 -> switchBlock10.ifIn;
                  connect condition10.out1 -> switchBlock10.condition;
                  connect in37 -> switchBlock10.elseIn;
                  connect switchBlock10.out1 -> switchBlock9.elseIn;
                  connect switchBlock9.out1 -> switchBlock7.elseIn;
                  connect switchBlock7.out1 -> switchBlock6.elseIn;
                  connect switchBlock6.out1 -> out2;
                  connect risingEdgeDetector1.out1Out1 -> condition11.in1;
                  connect cC_StartUpSetValue.out3 -> switchBlock11.ifIn;
                  connect condition11.out1 -> switchBlock11.condition;
                  connect risingEdgeDetector2.out1Out1 -> condition12.in1;
                  connect cC_OnSet_SetValue.out3 -> switchBlock12.ifIn;
                  connect condition12.out1 -> switchBlock12.condition;
                  connect in38 -> cC_ChangeSetValue.in33;
                  connect in39 -> cC_ChangeSetValue.in34;
                  connect in40 -> cC_ChangeSetValue.in35;
                  connect in41 -> cC_ChangeSetValue.in36;
                  connect cC_enabled_bIn2 -> condition13.in1;
                  connect cC_ChangeSetValue.out6 -> switchBlock13.ifIn;
                  connect condition13.out1 -> switchBlock13.condition;
                  connect in42 -> switchBlock13.elseIn;
                  connect switchBlock13.out1 -> cC_InitialSetValue.in4;
                  connect risingEdgeDetector.out1Out1 -> condition14.in1;
                  connect cC_InitialSetValue.out3 -> switchBlock14.ifIn;
                  connect condition14.out1 -> switchBlock14.condition;
                  connect in43 -> cC_ChangeSetValue.in37;
                  connect in44 -> cC_ChangeSetValue.in38;
                  connect in45 -> cC_ChangeSetValue.in39;
                  connect in46 -> cC_ChangeSetValue.in40;
                  connect cC_enabled_bIn2 -> condition15.in1;
                  connect cC_ChangeSetValue.out7 -> switchBlock15.ifIn;
                  connect condition15.out1 -> switchBlock15.condition;
                  connect in47 -> switchBlock15.elseIn;
                  connect switchBlock15.out1 -> switchBlock14.elseIn;
                  connect switchBlock14.out1 -> switchBlock12.elseIn;
                  connect switchBlock12.out1 -> switchBlock11.elseIn;
                  connect switchBlock11.out1 -> out3;
                  connect compareToZero1.yOut1 -> logicalOperator3.in2;
                  connect compareToZero.yOut1 -> logicalOperator3.in1;
                  connect rSFlipFlop.nOT_QOut2 -> terminator.in1;
                  connect rSFlipFlop.qOut1 -> logicalOperator2.in3;
                  connect logicalOperator6.out1 -> rSFlipFlop.rIn2;
                  connect logicalOperator5.out1 -> logicalOperator2.in1;
                  connect limiter_bIn1 -> logicalOperator5.in1;
                  connect edgeRising.yOut1 -> rSFlipFlop.sIn1;
                  connect logicalOperator4.out1 -> logicalOperator2.in2;
                  connect logicalOperator1.out1 -> edgeRising.rIn2;
                  connect falseBlock.yOut1 -> edgeRising.iVIn3;
                  connect logicalOperator3.out1 -> edgeRising.uIn1;
                  connect cC_enabled_bIn2 -> logicalOperator6.in1;
                  connect cC_enabled_bIn2 -> logicalOperator1.in1;
                  connect leverDown_StatIn6 -> cC_ChangeSetValue.leverDown_statIn2;
                  connect leverDown_StatIn6 -> compareToZero1.uIn1;
                  connect leverUp_StatIn5 -> compareToZero.uIn1;
                  connect leverUp_StatIn5 -> cC_ChangeSetValue.leverUp_statIn1;
                  connect cruiseControl_bIn3 -> logicalOperator.in1;
                  connect cruiseControl_bIn3 -> logicalOperator4.in1;
                  connect relOp.out1 -> logicalOperator.in2;
                  connect constant.out1 -> relOp.in2;
                  connect v_Vehicle_kmhIn4 -> cC_InitialSetValue.v_Vehicle_kmhIn1;
                  connect v_Vehicle_kmhIn4 -> cC_OnSet_SetValue.v_Vehicle_kmhIn1;
                  connect v_Vehicle_kmhIn4 -> relOp.in1;
                }
                <<Type="SubSystem">> component VERSION_INFO {
                  <<Type="SubSystem">> component Copyright {
                  }
                  component Copyright copyright;
                }
                component CC_Enabled cC_Enabled;
                component CC_On_Off cC_On_Off;
                component CC_SetValue cC_SetValue;
                component VERSION_INFO vERSION_INFO;
                connect in1 -> cC_SetValue.in1;
                connect in2 -> cC_SetValue.in2;
                connect in3 -> cC_SetValue.in3;
                connect in4 -> cC_SetValue.in4;
                connect in5 -> cC_SetValue.in5;
                connect in6 -> cC_SetValue.in6;
                connect in7 -> cC_SetValue.in7;
                connect in8 -> cC_SetValue.in8;
                connect in9 -> cC_SetValue.in9;
                connect in10 -> cC_SetValue.in10;
                connect in11 -> cC_SetValue.in11;
                connect in12 -> cC_SetValue.in12;
                connect in13 -> cC_SetValue.in13;
                connect in14 -> cC_SetValue.in14;
                connect in15 -> cC_SetValue.in15;
                connect in16 -> cC_SetValue.in16;
                connect in17 -> cC_SetValue.in17;
                connect in18 -> cC_SetValue.in18;
                connect in19 -> cC_SetValue.in19;
                connect in20 -> cC_SetValue.in20;
                connect in21 -> cC_SetValue.in21;
                connect in22 -> cC_SetValue.in22;
                connect in23 -> cC_SetValue.in23;
                connect in24 -> cC_SetValue.in24;
                connect in25 -> cC_SetValue.in25;
                connect in26 -> cC_SetValue.in26;
                connect in27 -> cC_SetValue.in27;
                connect cC_SetValue.out1 -> cC_Enabled.in1;
                connect in28 -> cC_SetValue.in28;
                connect in29 -> cC_SetValue.in29;
                connect in30 -> cC_SetValue.in30;
                connect in31 -> cC_SetValue.in31;
                connect in32 -> cC_SetValue.in32;
                connect in33 -> cC_SetValue.in33;
                connect in34 -> cC_SetValue.in34;
                connect in35 -> cC_SetValue.in35;
                connect in36 -> cC_SetValue.in36;
                connect in37 -> cC_SetValue.in37;
                connect cC_SetValue.out2 -> out1;
                connect in38 -> cC_SetValue.in38;
                connect in39 -> cC_SetValue.in39;
                connect in40 -> cC_SetValue.in40;
                connect in41 -> cC_SetValue.in41;
                connect in42 -> cC_SetValue.in42;
                connect in43 -> cC_SetValue.in43;
                connect in44 -> cC_SetValue.in44;
                connect in45 -> cC_SetValue.in45;
                connect in46 -> cC_SetValue.in46;
                connect in47 -> cC_SetValue.in47;
                connect cC_SetValue.out3 -> out2;
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
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Type="Switch">> component SwitchBlock {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock1 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition1 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition2 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock2 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
              }
              <<Type="Constant",Value="0">> component Constant1 {
                ports
                  out Double out1;
              }
              <<Condition="u1>0",Type="Condition">> component Condition3 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Type="Switch">> component SwitchBlock3 {
                ports
                  in Boolean ifIn,
                  in Boolean condition,
                  in Boolean elseIn,
                  out Boolean out1;
              }
              <<Type="Constant",Value="0">> component Constant2 {
                ports
                  out Boolean out1;
              }
              component Constant constant;
              component Tempomat_Function tempomat_Function;
              component VERSION_INFO vERSION_INFO;
              component SwitchBlock switchBlock;
              component Condition condition;
              component SwitchBlock1 switchBlock1;
              component Condition1 condition1;
              component Condition2 condition2;
              component SwitchBlock2 switchBlock2;
              component Constant1 constant1;
              component Condition3 condition3;
              component SwitchBlock3 switchBlock3;
              component Constant2 constant2;
              connect in1 -> tempomat_Function.in1;
              connect in2 -> tempomat_Function.in2;
              connect in3 -> tempomat_Function.in3;
              connect in4 -> tempomat_Function.in4;
              connect in5 -> tempomat_Function.in5;
              connect in6 -> tempomat_Function.in6;
              connect in7 -> tempomat_Function.in7;
              connect in8 -> tempomat_Function.in8;
              connect in9 -> tempomat_Function.in9;
              connect in10 -> tempomat_Function.in10;
              connect in11 -> tempomat_Function.in11;
              connect in12 -> tempomat_Function.in12;
              connect in13 -> tempomat_Function.in13;
              connect in14 -> tempomat_Function.in14;
              connect in15 -> tempomat_Function.in15;
              connect in16 -> tempomat_Function.in16;
              connect in17 -> tempomat_Function.in17;
              connect in18 -> tempomat_Function.in18;
              connect in19 -> tempomat_Function.in19;
              connect in20 -> tempomat_Function.in20;
              connect in21 -> tempomat_Function.in21;
              connect in22 -> tempomat_Function.in22;
              connect in23 -> tempomat_Function.in23;
              connect in24 -> tempomat_Function.in24;
              connect in25 -> tempomat_Function.in25;
              connect in26 -> tempomat_Function.in26;
              connect in27 -> tempomat_Function.in27;
              connect in28 -> tempomat_Function.in28;
              connect in29 -> tempomat_Function.in29;
              connect in30 -> tempomat_Function.in30;
              connect in31 -> tempomat_Function.in31;
              connect in32 -> tempomat_Function.in32;
              connect in33 -> tempomat_Function.in33;
              connect in34 -> tempomat_Function.in34;
              connect in35 -> tempomat_Function.in35;
              connect in36 -> tempomat_Function.in36;
              connect in37 -> tempomat_Function.in37;
              connect constant.out1 -> condition.in1;
              connect tempomat_Function.out1 -> switchBlock.ifIn;
              connect condition.out1 -> switchBlock.condition;
              connect in38 -> switchBlock.elseIn;
              connect switchBlock.out1 -> out1;
              connect in39 -> tempomat_Function.in38;
              connect in40 -> tempomat_Function.in39;
              connect in41 -> tempomat_Function.in40;
              connect in42 -> tempomat_Function.in41;
              connect in43 -> tempomat_Function.in42;
              connect in44 -> tempomat_Function.in43;
              connect in45 -> tempomat_Function.in44;
              connect in46 -> tempomat_Function.in45;
              connect in47 -> tempomat_Function.in46;
              connect in48 -> tempomat_Function.in47;
              connect constant.out1 -> condition1.in1;
              connect tempomat_Function.out2 -> switchBlock1.ifIn;
              connect condition1.out1 -> switchBlock1.condition;
              connect in49 -> switchBlock1.elseIn;
              connect switchBlock1.out1 -> out2;
              connect constant.out1 -> condition2.in1;
              connect condition2.out1 -> switchBlock2.condition;
              connect tempomat_Function.v_CC_delta_kmhOut1 -> switchBlock2.ifIn;
              connect constant1.out1 -> switchBlock2.elseIn;
              connect constant.out1 -> condition3.in1;
              connect condition3.out1 -> switchBlock3.condition;
              connect tempomat_Function.cC_active_bOut2 -> switchBlock3.ifIn;
              connect constant2.out1 -> switchBlock3.elseIn;
              connect switchBlock3.out1 -> cC_active_bOut2;
              connect switchBlock2.out1 -> v_CC_delta_kmhOut1;
              connect fTS_active_bIn8 -> tempomat_Function.fTS_active_bIn8;
              connect leverDown_StatIn7 -> tempomat_Function.leverDown_StatIn7;
              connect leverUp_StatIn6 -> tempomat_Function.leverUp_StatIn6;
              connect limiter_bIn5 -> tempomat_Function.limiter_bIn5;
              connect v_Vehicle_kmhIn4 -> tempomat_Function.v_Vehicle_kmhIn4;
              connect cruiseControl_bIn3 -> tempomat_Function.cruiseControl_bIn3;
              connect brakeForce_pedal_pcIn2 -> tempomat_Function.brakeForce_pedal_pcIn2;
              connect parkingBrake_bIn1 -> tempomat_Function.parkingBrake_bIn1;
            }
            <<Type="SubSystem">> component VERSION_INFO {
              <<Type="SubSystem">> component Copyright {
              }
              component Copyright copyright;
            }
            <<Type="SubSystem">> component VelocityControl {
              ports
                in Double brakeForceBoosted_pcIn1,
                in Double v_CC_delta_kmhIn2,
                in Double vMax_kmhIn3,
                in Double acceleration_pedal_pcIn4,
                in Double v_Vehicle_kmhIn5,
                in Double brakeForce_pedal_pcIn6,
                in Double brakeForce_Distronic_pcIn7,
                in Double decelerator_pcIn8,
                in Double brakeForce_Emergency_pcIn9,
                out Double acceleration_pcOut1,
                out Double brakeForce_pcOut2;
              <<Type="Constant",Value="0">> component Constant1 {
                ports
                  out Double out1;
              }
              <<Type="Constant",Value="0">> component Constant2 {
                ports
                  out Double out1;
              }
              <<Type="Constant",Value="0">> component Constant3 {
                ports
                  out Double out1;
              }
              <<Type="Constant",Value="0">> component Constant4 {
                ports
                  out Double out1;
              }
              <<Type="Constant",Value="100">> component Constant5 {
                ports
                  out Double out1;
              }
              <<Operator="AND",Type="Logic">> component LogOp {
                ports
                  in Boolean in1,
                  in Boolean in2,
                  out Boolean out1;
                effect in1 -> out1;
                effect in2 -> out1;
              }
              <<BreakpointsForDimension1="[0 1 5 10]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[0 20 60 100]",BreakpointsSpecification="Explicit values">> component LookUpTable {
                ports
                  in Double in1,
                  out Double out1;
                effect in1 -> out1;
              }
              <<BreakpointsForDimension1="[-10 -5 -1 0]",Type="Lookup_n-D",NumberOfTableDimensions="1",Table="[100 40 5 0]",BreakpointsSpecification="Explicit values">> component LookUpTable1 {
                ports
                  in Double in1,
                  out Double out1;
                effect in1 -> out1;
              }
              <<Function="max",Type="MinMax">> component MinMax1 {
                ports
                  in Double in1,
                  in Double in2,
                  out Double out1;
                effect in1 -> out1;
                effect in2 -> out1;
              }
              <<Function="max",Type="MinMax">> component MinMax2 {
                ports
                  in Double in1,
                  in Double in2,
                  in Double in3,
                  in Double in4,
                  in Double in5,
                  out Double brakeForce_pcOut1;
                effect in1 -> brakeForce_pcOut1;
                effect in2 -> brakeForce_pcOut1;
                effect in3 -> brakeForce_pcOut1;
                effect in4 -> brakeForce_pcOut1;
                effect in5 -> brakeForce_pcOut1;
              }
              <<Type="Product",Inputs="**/">> component Mul {
                ports
                  in Double in1,
                  in Double acceleration_pcIn2,
                  in Double in3,
                  out Double out1;
                effect in1 -> out1;
                effect acceleration_pcIn2 -> out1;
                effect in3 -> out1;
              }
              <<Operator=">",Type="RelationalOperator">> component RelOp {
                ports
                  in Double in1,
                  in Double in2,
                  out Boolean out1;
                effect in1 -> out1;
                effect in2 -> out1;
              }
              <<Operator=">",Type="RelationalOperator">> component RelOp1 {
                ports
                  in Double in1,
                  in Double in2,
                  out Boolean out1;
                effect in1 -> out1;
                effect in2 -> out1;
              }
              <<Type="Saturate",LowerLimit="0",UpperLimit="10">> component Saturation {
                ports
                  in Double in1,
                  out Double out1;
                effect in1 -> out1;
              }
              <<Type="Saturate",LowerLimit="-10",UpperLimit="0">> component Saturation1 {
                ports
                  in Double in1,
                  out Double out1;
                effect in1 -> out1;
              }
              <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double acceleration_pcOut1;
                effect ifIn -> acceleration_pcOut1;
                effect condition -> acceleration_pcOut1;
                effect elseIn -> acceleration_pcOut1;
              }
              <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock1 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double out1;
                effect ifIn -> out1;
                effect condition -> out1;
                effect elseIn -> out1;
              }
              <<Type="Switch",Criteria="u2 > Threshold",Threshold="1">> component SwitchBlock2 {
                ports
                  in Double ifIn,
                  in Boolean condition,
                  in Double elseIn,
                  out Double acceleration_pcOut1;
                effect ifIn -> acceleration_pcOut1;
                effect condition -> acceleration_pcOut1;
                effect elseIn -> acceleration_pcOut1;
              }
              <<Type="SubSystem">> component VERSION_INFO {
                <<Type="SubSystem">> component Copyright {
                }
                component Copyright copyright;
              }
              <<Condition="u2 > 1",Type="Condition">> component Condition {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
                ports
                  in Boolean in1,
                  out Boolean out1;
              }
              <<Condition="u2 > 0",Type="Condition">> component Condition2 {
                ports
                  in Double in1,
                  out Boolean out1;
              }
              component Constant1 constant1;
              component Constant2 constant2;
              component Constant3 constant3;
              component Constant4 constant4;
              component Constant5 constant5;
              component LogOp logOp;
              component LookUpTable lookUpTable;
              component LookUpTable1 lookUpTable1;
              component MinMax1 minMax1;
              component MinMax2 minMax2;
              component Mul mul;
              component RelOp relOp;
              component RelOp1 relOp1;
              component Saturation saturation;
              component Saturation1 saturation1;
              component SwitchBlock switchBlock;
              component SwitchBlock1 switchBlock1;
              component SwitchBlock2 switchBlock2;
              component VERSION_INFO vERSION_INFO;
              component Condition condition;
              component Condition1 condition1;
              component Condition2 condition2;
              connect condition2.out1 -> switchBlock1.condition;
              connect condition.out1 -> switchBlock2.condition;
              connect condition1.out1 -> switchBlock.condition;
              connect minMax2.brakeForce_pcOut1 -> condition.in1;
              connect logOp.out1 -> condition1.in1;
              connect minMax1.out1 -> condition2.in1;
              connect constant5.out1 -> mul.in3;
              connect mul.out1 -> switchBlock2.elseIn;
              connect decelerator_pcIn8 -> mul.in1;
              connect constant3.out1 -> switchBlock2.ifIn;
              connect switchBlock.acceleration_pcOut1 -> mul.acceleration_pcIn2;
              connect brakeForce_Distronic_pcIn7 -> minMax2.in4;
              connect brakeForce_Emergency_pcIn9 -> minMax2.in5;
              connect constant4.out1 -> switchBlock1.ifIn;
              connect minMax1.out1 -> switchBlock.elseIn;
              connect switchBlock1.out1 -> minMax2.in1;
              connect minMax2.brakeForce_pcOut1 -> brakeForce_pcOut2;
              connect brakeForce_pedal_pcIn6 -> minMax2.in3;
              connect lookUpTable1.out1 -> switchBlock1.elseIn;
              connect brakeForceBoosted_pcIn1 -> minMax2.in2;
              connect saturation1.out1 -> lookUpTable1.in1;
              connect saturation.out1 -> lookUpTable.in1;
              connect constant2.out1 -> relOp1.in2;
              connect switchBlock2.acceleration_pcOut1 -> acceleration_pcOut1;
              connect acceleration_pedal_pcIn4 -> minMax1.in1;
              connect vMax_kmhIn3 -> relOp.in2;
              connect vMax_kmhIn3 -> relOp1.in1;
              connect v_Vehicle_kmhIn5 -> relOp.in1;
              connect lookUpTable.out1 -> minMax1.in2;
              connect v_CC_delta_kmhIn2 -> saturation.in1;
              connect v_CC_delta_kmhIn2 -> saturation1.in1;
              connect relOp.out1 -> logOp.in1;
              connect relOp1.out1 -> logOp.in2;
              connect constant1.out1 -> switchBlock.ifIn;
            }
            <<Type="UnitDelay",InitialCondition="-1">> component UnitDelay {
              ports
                in Double valueIn,
                out Double valueOut;
            }
            <<Type="UnitDelay",InitialCondition="0">> component UnitDelay1 {
              ports
                in Double valueIn,
                out Double valueOut;
            }
            component BrakeAssistant brakeAssistant;
            component Distancewarner distancewarner;
            component Distronic distronic;
            component EmergencyBrake emergencyBrake;
            component FAS_Input fAS_Input;
            component FollowToStop followToStop;
            component Limiter limiter;
            component LogOp logOp;
            component Tempomat tempomat;
            component VERSION_INFO vERSION_INFO;
            component VelocityControl velocityControl;
            component UnitDelay unitDelay;
            component UnitDelay1 unitDelay1;
            connect tempomat.cC_active_bOut2 -> distronic.cC_active_bIn1;
            connect distronic.brakeForce_Distronic_pcOut1 -> velocityControl.brakeForce_Distronic_pcIn7;
            connect distronic.decelerator_pcOut2 -> velocityControl.decelerator_pcIn8;
            connect followToStop.fTS_Abstand_soll_mOut2 -> distronic.fTS_Abstand_soll_mIn7;
            connect followToStop.fTS_active_bOut1 -> tempomat.fTS_active_bIn8;
            connect followToStop.fTS_active_bOut1 -> distronic.fTS_active_bIn6;
            connect emergencyBrake.brakeForce_Emergency_pcOut1 -> velocityControl.brakeForce_Emergency_pcIn9;
            connect emergencyBrake.acusti_warn_bOut2 -> logOp.acusti_warn_bIn2;
            connect fAS_Input.v_Vehicle_kmh2 -> followToStop.v_Vehicle_kmhIn1;
            connect fAS_Input.v_Obj_rel_kmh4 -> followToStop.v_Obj_rel_kmhIn2;
            connect fAS_Input.v_Vehicle_ms2 -> emergencyBrake.v_Vehicle_msIn1;
            connect fAS_Input.v_Obj_rel_ms -> emergencyBrake.v_Obj_rel_msIn3;
            connect fAS_Input.distance_Object_m2 -> emergencyBrake.distance_Object_mIn2;
            connect fAS_Input.v_Obj_rel_kmh -> distronic.v_Obj_rel_kmhIn5;
            connect fAS_Input.distance_stat -> distronic.distance_statIn4;
            connect fAS_Input.v_Vehicle_ms1 -> distronic.v_Vehicle_msIn2;
            connect fAS_Input.distance_Object_m1 -> distronic.distance_Object_mIn3;
            connect distancewarner.acusti_warn_bOut2 -> logOp.acusti_warn_bIn1;
            connect fAS_Input.distance_Object_m -> distancewarner.distance_Object_mIn2;
            connect fAS_Input.v_Vehicle_ms -> distancewarner.v_Vehicle_msIn1;
            connect fAS_Input.v_Sign_kmh1 -> limiter.v_Sign_kmhIn6;
            connect fAS_Input.sign_b1 -> limiter.sign_bIn7;
            connect fAS_Input.brakeForce_pedal_pc1 -> velocityControl.brakeForce_pedal_pcIn6;
            connect fAS_Input.brakeForce_pedal_pc2 -> brakeAssistant.brakeForce_pedal_pcIn1;
            connect fAS_Input.limiter_b2 -> tempomat.limiter_bIn5;
            connect limiter.vMax_kmhOut1 -> velocityControl.vMax_kmhIn3;
            connect tempomat.v_CC_delta_kmhOut1 -> velocityControl.v_CC_delta_kmhIn2;
            connect fAS_Input.acceleration_pedal_pc2 -> limiter.acceleration_pedal_pcIn2;
            connect fAS_Input.acceleration_pedal_pc1 -> velocityControl.acceleration_pedal_pcIn4;
            connect fAS_Input.v_Vehicle_kmh3 -> velocityControl.v_Vehicle_kmhIn5;
            connect fAS_Input.leverUp_stat1 -> limiter.leverUp_statIn4;
            connect fAS_Input.v_Vehicle_kmh4 -> limiter.v_Vehicle_kmhIn3;
            connect fAS_Input.limiter_b1 -> limiter.limiter_bIn1;
            connect fAS_Input.leverDown_stat1 -> limiter.leverDown_statIn5;
            connect fAS_Input.cruiseControl_b1 -> tempomat.cruiseControl_bIn3;
            connect fAS_Input.v_Vehicle_kmh5 -> tempomat.v_Vehicle_kmhIn4;
            connect fAS_Input.leverDown_stat2 -> tempomat.leverDown_StatIn7;
            connect fAS_Input.leverUp_stat2 -> tempomat.leverUp_StatIn6;
            connect fAS_Input.brakeForce_pedal_pc3 -> tempomat.brakeForce_pedal_pcIn2;
            connect fAS_Input.parkingBrake_b1 -> tempomat.parkingBrake_bIn1;
            connect unitDelay.valueOut -> tempomat.in1;
            connect unitDelay.valueOut -> tempomat.in2;
            connect unitDelay.valueOut -> tempomat.in3;
            connect unitDelay.valueOut -> tempomat.in4;
            connect unitDelay.valueOut -> tempomat.in5;
            connect unitDelay.valueOut -> tempomat.in6;
            connect unitDelay.valueOut -> tempomat.in7;
            connect unitDelay.valueOut -> tempomat.in8;
            connect unitDelay.valueOut -> tempomat.in9;
            connect unitDelay.valueOut -> tempomat.in10;
            connect unitDelay.valueOut -> tempomat.in11;
            connect unitDelay.valueOut -> tempomat.in12;
            connect unitDelay.valueOut -> tempomat.in13;
            connect unitDelay.valueOut -> tempomat.in14;
            connect unitDelay.valueOut -> tempomat.in15;
            connect unitDelay.valueOut -> tempomat.in16;
            connect unitDelay.valueOut -> tempomat.in17;
            connect unitDelay.valueOut -> tempomat.in18;
            connect unitDelay.valueOut -> tempomat.in19;
            connect unitDelay.valueOut -> tempomat.in20;
            connect unitDelay.valueOut -> tempomat.in21;
            connect unitDelay.valueOut -> tempomat.in22;
            connect unitDelay.valueOut -> tempomat.in23;
            connect unitDelay.valueOut -> tempomat.in24;
            connect unitDelay.valueOut -> tempomat.in25;
            connect unitDelay.valueOut -> tempomat.in26;
            connect unitDelay.valueOut -> tempomat.in27;
            connect unitDelay.valueOut -> tempomat.in28;
            connect unitDelay.valueOut -> tempomat.in29;
            connect unitDelay.valueOut -> tempomat.in30;
            connect unitDelay.valueOut -> tempomat.in31;
            connect unitDelay.valueOut -> tempomat.in32;
            connect unitDelay.valueOut -> tempomat.in33;
            connect unitDelay.valueOut -> tempomat.in34;
            connect unitDelay.valueOut -> tempomat.in35;
            connect unitDelay.valueOut -> tempomat.in36;
            connect unitDelay.valueOut -> tempomat.in37;
            connect unitDelay.valueOut -> tempomat.in38;
            connect tempomat.out1 -> limiter.in1;
            connect unitDelay.valueOut -> tempomat.in39;
            connect unitDelay.valueOut -> tempomat.in40;
            connect unitDelay.valueOut -> tempomat.in41;
            connect unitDelay.valueOut -> tempomat.in42;
            connect unitDelay.valueOut -> tempomat.in43;
            connect unitDelay.valueOut -> tempomat.in44;
            connect unitDelay.valueOut -> tempomat.in45;
            connect unitDelay.valueOut -> tempomat.in46;
            connect unitDelay.valueOut -> tempomat.in47;
            connect unitDelay.valueOut -> tempomat.in48;
            connect unitDelay.valueOut -> tempomat.in49;
            connect tempomat.out2 -> limiter.in2;
            connect limiter.out1 -> unitDelay.valueIn;
            connect unitDelay1.valueOut -> limiter.in3;
            connect unitDelay1.valueOut -> limiter.in4;
            connect unitDelay1.valueOut -> limiter.in5;
            connect unitDelay1.valueOut -> limiter.in6;
            connect unitDelay1.valueOut -> limiter.in7;
            connect unitDelay1.valueOut -> limiter.in8;
            connect unitDelay1.valueOut -> limiter.in9;
            connect limiter.out2 -> unitDelay1.valueIn;
            connect parkingBrake_b -> fAS_Input.parkingBrake_b;
            connect brakeForce_pedal_pc -> fAS_Input.brakeForce_pedal_pc;
            connect acceleration_pedal_pc -> fAS_Input.acceleration_pedal_pc;
            connect cruiseControl_b -> fAS_Input.cruiseControl_b;
            connect limiter_b -> fAS_Input.limiter_b;
            connect leverUp_stat -> fAS_Input.leverUp_stat;
            connect leverDown_stat -> fAS_Input.leverDown_stat;
            connect v_Vehicle_kmh -> fAS_Input.v_Vehicle_kmh;
            connect v_Vehicle_kmh1 -> fAS_Input.v_Vehicle_kmh1;
            connect v_Sign_kmh -> fAS_Input.v_Sign_kmh;
            connect sign_b -> fAS_Input.sign_b;
            connect distance_stat -> fAS_Input.distance_stat1;
            connect distance_stat1 -> fAS_Input.distance_stat2;
            connect v_Obj_rel_kmh -> fAS_Input.v_Obj_rel_kmh1;
            connect v_Obj_rel_kmh1 -> fAS_Input.v_Obj_rel_kmh2;
            connect v_Obj_rel_kmh2 -> fAS_Input.v_Obj_rel_kmh3;
            connect distance_Object_m -> fAS_Input.distance_Object_m3;
            connect distance_Object_m1 -> fAS_Input.distance_Object_m4;
            connect distance_Object_m2 -> fAS_Input.distance_Object_m5;
            connect distance_Object_m3 -> fAS_Input.distance_Object_m6;
            connect tempomat.cC_active_bOut2 -> cC_active_b;
            connect velocityControl.acceleration_pcOut1 -> acceleration_pc;
            connect velocityControl.brakeForce_pcOut2 -> brakeForce_pc;
            connect unitDelay.valueOut -> cCSetValue_kmh;
            connect unitDelay1.valueOut -> limiterSetValue_kmh;
            connect limiter.limiter_active_bOut2 -> limiter_active_b;
            connect distancewarner.opti_warn_bOut1 -> opti_warn_b;
            connect logOp.acusti_warn_bOut1 -> acusti_warn_b;
            connect brakeAssistant.brakeForceBoosted_pcOut1 -> velocityControl.brakeForceBoosted_pcIn1;
          }
          <<Type="SubSystem">> component DEMO_FAS_Input {
            ports
              in Boolean parkingBrake_bIn1,
              in Double brakeForce_pedal_pcIn2,
              in Double _Acceleration_pedal_pcIn3,
              in Boolean cruiseControl_bIn4,
              in Boolean limiter_bIn5,
              in Double leverUp_statIn6,
              in Double leverDown_statIn7,
              in Double v_Vehicle_bIn8,
              in Double v_Sign_kmhIn9,
              in Boolean sign_bIn10,
              in Double distance_statIn11,
              in Double v_Obj_rel_kmhIn12,
              in Double distance_Object_mIn13,
              out Boolean parkingBrake_b,
              out Double brakeForce_pedal_pc,
              out Double acceleration_pedal_pc,
              out Boolean cruiseControl_b,
              out Boolean limiter_b,
              out Double leverUp_stat,
              out Double leverDown_stat,
              out Double v_Vehicle_kmh,
              out Double v_Vehicle_kmh1,
              out Double v_Sign_kmh,
              out Boolean sign_b,
              out Double distance_stat,
              out Double distance_stat1,
              out Double v_Obj_rel_kmh,
              out Double v_Obj_rel_kmh1,
              out Double v_Obj_rel_kmh2,
              out Double distance_Object_m,
              out Double distance_Object_m1,
              out Double distance_Object_m2,
              out Double distance_Object_m3;
            connect parkingBrake_bIn1 -> parkingBrake_b;
            connect brakeForce_pedal_pcIn2 -> brakeForce_pedal_pc;
            connect _Acceleration_pedal_pcIn3 -> acceleration_pedal_pc;
            connect cruiseControl_bIn4 -> cruiseControl_b;
            connect limiter_bIn5 -> limiter_b;
            connect leverUp_statIn6 -> leverUp_stat;
            connect leverDown_statIn7 -> leverDown_stat;
            connect v_Vehicle_bIn8 -> v_Vehicle_kmh;
            connect v_Vehicle_bIn8 -> v_Vehicle_kmh1;
            connect v_Sign_kmhIn9 -> v_Sign_kmh;
            connect sign_bIn10 -> sign_b;
            connect distance_statIn11 -> distance_stat;
            connect distance_statIn11 -> distance_stat1;
            connect v_Obj_rel_kmhIn12 -> v_Obj_rel_kmh;
            connect v_Obj_rel_kmhIn12 -> v_Obj_rel_kmh1;
            connect v_Obj_rel_kmhIn12 -> v_Obj_rel_kmh2;
            connect distance_Object_mIn13 -> distance_Object_m;
            connect distance_Object_mIn13 -> distance_Object_m1;
            connect distance_Object_mIn13 -> distance_Object_m2;
            connect distance_Object_mIn13 -> distance_Object_m3;
          }
          <<Type="SubSystem">> component DEMO_FAS_Output {
            ports
              out Boolean cC_active_bOut1,
              out Double acceleration_pcOut2,
              out Double brakeForce_pcOut3,
              out Double cCSetValue_kmhOut4,
              out Double limiterSetValue_kmhOut5,
              out Boolean limiter_active_bOut6,
              out Boolean opti_warn_bOut7,
              out Boolean acusti_warn_bOut8,
              in Boolean cC_active_b,
              in Double acceleration_pc,
              in Double brakeForce_pc,
              in Double cCSetValue_kmh,
              in Double limiterSetValue_kmh,
              in Boolean limiter_active_b,
              in Boolean opti_warn_b,
              in Boolean acusti_warn_b;
            connect cC_active_b -> cC_active_bOut1;
            connect acceleration_pc -> acceleration_pcOut2;
            connect brakeForce_pc -> brakeForce_pcOut3;
            connect cCSetValue_kmh -> cCSetValue_kmhOut4;
            connect limiterSetValue_kmh -> limiterSetValue_kmhOut5;
            connect limiter_active_b -> limiter_active_bOut6;
            connect opti_warn_b -> opti_warn_bOut7;
            connect acusti_warn_b -> acusti_warn_bOut8;
          }
          component DEMO_FAS_Funktion dEMO_FAS_Funktion;
          component DEMO_FAS_Input dEMO_FAS_Input;
          component DEMO_FAS_Output dEMO_FAS_Output;
          connect dEMO_FAS_Input.parkingBrake_b -> dEMO_FAS_Funktion.parkingBrake_b;
          connect dEMO_FAS_Input.brakeForce_pedal_pc -> dEMO_FAS_Funktion.brakeForce_pedal_pc;
          connect dEMO_FAS_Input.acceleration_pedal_pc -> dEMO_FAS_Funktion.acceleration_pedal_pc;
          connect dEMO_FAS_Input.cruiseControl_b -> dEMO_FAS_Funktion.cruiseControl_b;
          connect dEMO_FAS_Input.limiter_b -> dEMO_FAS_Funktion.limiter_b;
          connect dEMO_FAS_Input.leverUp_stat -> dEMO_FAS_Funktion.leverUp_stat;
          connect dEMO_FAS_Input.leverDown_stat -> dEMO_FAS_Funktion.leverDown_stat;
          connect dEMO_FAS_Input.v_Vehicle_kmh -> dEMO_FAS_Funktion.v_Vehicle_kmh;
          connect dEMO_FAS_Input.v_Vehicle_kmh1 -> dEMO_FAS_Funktion.v_Vehicle_kmh1;
          connect dEMO_FAS_Input.v_Sign_kmh -> dEMO_FAS_Funktion.v_Sign_kmh;
          connect dEMO_FAS_Input.sign_b -> dEMO_FAS_Funktion.sign_b;
          connect dEMO_FAS_Input.distance_stat -> dEMO_FAS_Funktion.distance_stat;
          connect dEMO_FAS_Input.distance_stat1 -> dEMO_FAS_Funktion.distance_stat1;
          connect dEMO_FAS_Input.v_Obj_rel_kmh -> dEMO_FAS_Funktion.v_Obj_rel_kmh;
          connect dEMO_FAS_Input.v_Obj_rel_kmh1 -> dEMO_FAS_Funktion.v_Obj_rel_kmh1;
          connect dEMO_FAS_Input.v_Obj_rel_kmh2 -> dEMO_FAS_Funktion.v_Obj_rel_kmh2;
          connect dEMO_FAS_Input.distance_Object_m -> dEMO_FAS_Funktion.distance_Object_m;
          connect dEMO_FAS_Input.distance_Object_m1 -> dEMO_FAS_Funktion.distance_Object_m1;
          connect dEMO_FAS_Input.distance_Object_m2 -> dEMO_FAS_Funktion.distance_Object_m2;
          connect dEMO_FAS_Input.distance_Object_m3 -> dEMO_FAS_Funktion.distance_Object_m3;
          connect dEMO_FAS_Funktion.cC_active_b -> dEMO_FAS_Output.cC_active_b;
          connect dEMO_FAS_Funktion.acceleration_pc -> dEMO_FAS_Output.acceleration_pc;
          connect dEMO_FAS_Funktion.brakeForce_pc -> dEMO_FAS_Output.brakeForce_pc;
          connect dEMO_FAS_Funktion.cCSetValue_kmh -> dEMO_FAS_Output.cCSetValue_kmh;
          connect dEMO_FAS_Funktion.limiterSetValue_kmh -> dEMO_FAS_Output.limiterSetValue_kmh;
          connect dEMO_FAS_Funktion.limiter_active_b -> dEMO_FAS_Output.limiter_active_b;
          connect dEMO_FAS_Funktion.opti_warn_b -> dEMO_FAS_Output.opti_warn_b;
          connect dEMO_FAS_Funktion.acusti_warn_b -> dEMO_FAS_Output.acusti_warn_b;
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
          connect _Acceleration_pedal_pcIn3 -> dEMO_FAS_Input._Acceleration_pedal_pcIn3;
          connect _Limiter_bIn5 -> dEMO_FAS_Input.limiter_bIn5;
          connect _LeverUp_statIn6 -> dEMO_FAS_Input.leverUp_statIn6;
          connect _LeverDown_statIn7 -> dEMO_FAS_Input.leverDown_statIn7;
          connect _V_Vehicle_kmhIn8 -> dEMO_FAS_Input.v_Vehicle_bIn8;
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
        connect _Acceleration_pedal_pcIn3 -> dEMO_FAS._Acceleration_pedal_pcIn3;
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
      connect _Acceleration_pedal_pcIn3 -> subsystem._Acceleration_pedal_pcIn3;
      connect _BrakeForce_pedal_pcIn2 -> subsystem._BrakeForce_pedal_pcIn2;
      connect _ParkingBrake_bIn1 -> subsystem._ParkingBrake_bIn1;
    }
    component DEMO_FAS dEMO_FAS;
    connect distance_stat -> dEMO_FAS._Distance_statIn11;
    connect v_Vehicle_kmh -> dEMO_FAS._V_Vehicle_kmhIn8;
    connect parkingBrake_b -> dEMO_FAS._ParkingBrake_bIn1;
    connect leverUp_stat -> dEMO_FAS._LeverUp_statIn6;
    connect leverDown_stat -> dEMO_FAS._LeverDown_statIn7;
    connect brakeForce_pedal_pc -> dEMO_FAS._BrakeForce_pedal_pcIn2;
    connect limiter_b -> dEMO_FAS._Limiter_bIn5;
    connect cruiseControl_b -> dEMO_FAS._CruiseControl_bIn4;
    connect acceleration_pedal_pc -> dEMO_FAS._Acceleration_pedal_pcIn3;
    connect v_Sign_kmh -> dEMO_FAS._V_Sign_kmhIn9;
    connect sign_b -> dEMO_FAS._Sign_bIn10;
    connect v_Obj_rel_kmh -> dEMO_FAS._V_Obj_rel_kmhIn12;
    connect dEMO_FAS._CC_active_bOut1 -> cC_active_b;
    connect dEMO_FAS._CCSetValue_kmhOut4 -> cCSetValue_kmh;
    connect dEMO_FAS._LimiterSetValue_kmhOut5 -> limiterSetValue_kmh;
    connect dEMO_FAS._Limiter_active_bOut6 -> limiter_active_b;
    connect dEMO_FAS._Opti_warn_bOut7 -> opti_warn_b;
    connect dEMO_FAS._Acusti_warn_bOut8 -> acusti_warn_b;
    connect dEMO_FAS._BrakeForce_pcOut3 -> brakeForce_pc;
    connect dEMO_FAS._Acceleration_pcOut2 -> acceleration_pc;
  }
  <<Type="SubSystem">> component SimToRealTime {
    <<Type="Constant",Value="0">> component Constant {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant1 {
      ports
        out Double out1;
    }
    <<Type="Terminator">> component Terminator {
      ports
        in Double in1;
    }
    <<Type="Terminator">> component Terminator1 {
      ports
        in Double in1;
    }
    <<Type="Terminator">> component Terminator2 {
      ports
        in Double in1;
    }
    <<Type="Terminator">> component Terminator3 {
      ports
        in Double in1;
    }
    <<Type="Terminator">> component Terminator4 {
      ports
        in Double in1;
    }
    <<Type="Terminator">> component Terminator5 {
      ports
        in Double in1;
    }
    <<Type="SubSystem">> component VAPS_TimeConfiguration {
      ports
        in Double timeConfigIn1,
        in Double maxSpeedIn2,
        out Double factor_SimTimeOut1,
        out Double timeConfig_ConsumerOut2,
        out Double timeConfig_ProducerOut3,
        out Double tsSimOut4,
        out Double tsRTimeOut5,
        out Double tsRTimeEffOut6;
      <<Type="Constant",Value="0">> component Constant1 {
        ports
          out Double out1;
      }
      <<Type="Constant",Value="2">> component Constant10 {
        ports
          out Double out1;
      }
      <<Type="Constant",Value="0">> component Constant4 {
        ports
          out Double out1;
      }
      <<Type="SubSystem">> component RealTimeTimerAdapter {
        ports
          in Double in1In1,
          out Double out1Out1;
        <<Type="Constant",Value="-10">> component Constant1 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="1">> component Constant10 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="1">> component Constant11 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="-1">> component Constant12 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="1">> component Constant13 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="1.1">> component Constant15 {
          ports
            out Double out1;
        }
        <<Type="Product",Inputs="*/">> component Div1 {
          ports
            in Double in1,
            in Double in2,
            out Double out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator="AND",Type="Logic">> component LogOp1 {
          ports
            in Boolean in1,
            in Boolean in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator=">",Type="RelationalOperator">> component RelOp2 {
          ports
            in Double in1,
            in Double in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator="<=",Type="RelationalOperator">> component RelOp3 {
          ports
            in Double in1,
            in Double in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator=">=",Type="RelationalOperator">> component RelOp4 {
          ports
            in Double in1,
            in Double in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Type="Sum",ListOfSigns="+-">> component Sum {
          ports
            in Double in1,
            in Double in2,
            out Double out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock2 {
          ports
            in Double ifIn,
            in Boolean condition,
            in Double elseIn,
            out Double out1;
          effect ifIn -> out1;
          effect condition -> out1;
          effect elseIn -> out1;
        }
        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock3 {
          ports
            in Double ifIn,
            in Boolean condition,
            in Double elseIn,
            out Double out1;
          effect ifIn -> out1;
          effect condition -> out1;
          effect elseIn -> out1;
        }
        <<Condition="u2 >= 1",Type="Condition">> component Condition {
          ports
            in Boolean in1,
            out Boolean out1;
        }
        <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
          ports
            in Boolean in1,
            out Boolean out1;
        }
        component Constant1 constant1;
        component Constant10 constant10;
        component Constant11 constant11;
        component Constant12 constant12;
        component Constant13 constant13;
        component Constant15 constant15;
        component Div1 div1;
        component LogOp1 logOp1;
        component RelOp2 relOp2;
        component RelOp3 relOp3;
        component RelOp4 relOp4;
        component Sum sum;
        component SwitchBlock2 switchBlock2;
        component SwitchBlock3 switchBlock3;
        component Condition condition;
        component Condition1 condition1;
        connect condition1.out1 -> switchBlock2.condition;
        connect condition.out1 -> switchBlock3.condition;
        connect logOp1.out1 -> condition.in1;
        connect relOp2.out1 -> condition1.in1;
        connect in1In1 -> relOp2.in1;
        connect in1In1 -> switchBlock2.ifIn;
        connect in1In1 -> relOp3.in1;
        connect in1In1 -> relOp4.in1;
        connect in1In1 -> div1.in1;
        connect constant10.out1 -> relOp2.in2;
        connect switchBlock2.out1 -> out1Out1;
        connect constant11.out1 -> relOp3.in2;
        connect switchBlock3.out1 -> switchBlock2.elseIn;
        connect constant12.out1 -> relOp4.in2;
        connect relOp3.out1 -> logOp1.in1;
        connect relOp4.out1 -> logOp1.in2;
        connect constant13.out1 -> switchBlock3.ifIn;
        connect sum.out1 -> switchBlock3.elseIn;
        connect constant1.out1 -> div1.in2;
        connect constant15.out1 -> sum.in1;
        connect div1.out1 -> sum.in2;
      }
      <<Type="SubSystem">> component RealTimeTimerAdapter_Value {
        ports
          in Double in1In1,
          out Double out1Out1;
        <<Type="Constant",Value="1">> component Constant11 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="-1">> component Constant12 {
          ports
            out Double out1;
        }
        <<Type="Constant",Value="0">> component Constant13 {
          ports
            out Double out1;
        }
        <<Operator="AND",Type="Logic">> component LogOp1 {
          ports
            in Boolean in1,
            in Boolean in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator="<=",Type="RelationalOperator">> component RelOp3 {
          ports
            in Double in1,
            in Double in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Operator=">=",Type="RelationalOperator">> component RelOp4 {
          ports
            in Double in1,
            in Double in2,
            out Boolean out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock3 {
          ports
            in Double ifIn,
            in Boolean condition,
            in Double elseIn,
            out Double out1;
          effect ifIn -> out1;
          effect condition -> out1;
          effect elseIn -> out1;
        }
        <<Condition="u2 >= 1",Type="Condition">> component Condition {
          ports
            in Boolean in1,
            out Boolean out1;
        }
        component Constant11 constant11;
        component Constant12 constant12;
        component Constant13 constant13;
        component LogOp1 logOp1;
        component RelOp3 relOp3;
        component RelOp4 relOp4;
        component SwitchBlock3 switchBlock3;
        component Condition condition;
        connect condition.out1 -> switchBlock3.condition;
        connect logOp1.out1 -> condition.in1;
        connect constant13.out1 -> switchBlock3.ifIn;
        connect relOp4.out1 -> logOp1.in2;
        connect relOp3.out1 -> logOp1.in1;
        connect constant12.out1 -> relOp4.in2;
        connect constant11.out1 -> relOp3.in2;
        connect switchBlock3.out1 -> out1Out1;
        connect in1In1 -> switchBlock3.elseIn;
        connect in1In1 -> relOp4.in1;
        connect in1In1 -> relOp3.in1;
      }
      <<Operator="==",Type="RelationalOperator">> component RelOp4 {
        ports
          in Double in1,
          in Double in2,
          out Boolean out1;
        effect in1 -> out1;
        effect in2 -> out1;
      }
      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock1 {
        ports
          in Double ifIn,
          in Boolean condition,
          in Double elseIn,
          out Double out1;
        effect ifIn -> out1;
        effect condition -> out1;
        effect elseIn -> out1;
      }
      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock2 {
        ports
          in Double ifIn,
          in Boolean condition,
          in Double elseIn,
          out Double out1;
        effect ifIn -> out1;
        effect condition -> out1;
        effect elseIn -> out1;
      }
      <<Type="SubSystem">> component SysInit1 {
        ports
          out Double yOut1;
        <<Type="UnitDelay",InitialCondition="1">> component Memory_Init {
          ports
            in Double in1,
            out Double out1;
          effect in1 -> out1;
        }
        <<Type="Constant",Value="0">> component Zero_Init {
          ports
            out Double out1;
        }
        component Memory_Init memory_Init;
        component Zero_Init zero_Init;
        connect memory_Init.out1 -> yOut1;
        connect zero_Init.out1 -> memory_Init.in1;
      }
      <<Type="SubSystem">> component VAPSRealTimeTimer {
        ports
          in Double timeFactorIn1,
          out Double tsSimOut1,
          out Double tsRTimeOut2,
          out Double tsRTimeEffectiveOut3;
        <<Type="M-S-Function">> component Level2MfileSFunction {
          ports
            in Double in1,
            out Double tsSimOut1,
            out Double tsRTimeOut2,
            out Double tsRTimeEffectiveOut3;
          effect in1 -> tsSimOut1;
          effect in1 -> tsRTimeOut2;
          effect in1 -> tsRTimeEffectiveOut3;
        }
        component Level2MfileSFunction level2MfileSFunction;
        connect timeFactorIn1 -> level2MfileSFunction.in1;
        connect level2MfileSFunction.tsSimOut1 -> tsSimOut1;
        connect level2MfileSFunction.tsRTimeOut2 -> tsRTimeOut2;
        connect level2MfileSFunction.tsRTimeEffectiveOut3 -> tsRTimeEffectiveOut3;
      }
      <<Condition="u2 >= 1",Type="Condition">> component Condition {
        ports
          in Boolean in1,
          out Boolean out1;
      }
      <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
        ports
          in Double in1,
          out Boolean out1;
      }
      component Constant1 constant1;
      component Constant10 constant10;
      component Constant4 constant4;
      component RealTimeTimerAdapter realTimeTimerAdapter;
      component RealTimeTimerAdapter_Value realTimeTimerAdapter_Value;
      component RelOp4 relOp4;
      component SwitchBlock1 switchBlock1;
      component SwitchBlock2 switchBlock2;
      component SysInit1 sysInit1;
      component VAPSRealTimeTimer vAPSRealTimeTimer;
      component Condition condition;
      component Condition1 condition1;
      connect condition.out1 -> switchBlock1.condition;
      connect condition1.out1 -> switchBlock2.condition;
      connect relOp4.out1 -> condition.in1;
      connect sysInit1.yOut1 -> condition1.in1;
      connect constant1.out1 -> switchBlock1.ifIn;
      connect maxSpeedIn2 -> relOp4.in1;
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
      connect constant4.out1 -> switchBlock2.ifIn;
      connect switchBlock2.out1 -> realTimeTimerAdapter_Value.in1In1;
      connect switchBlock2.out1 -> realTimeTimerAdapter.in1In1;
    }
    component Constant constant;
    component Constant1 constant1;
    component Terminator terminator;
    component Terminator1 terminator1;
    component Terminator2 terminator2;
    component Terminator3 terminator3;
    component Terminator4 terminator4;
    component Terminator5 terminator5;
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
  <<Type="SubSystem">> component Umgebung {
    ports
      in Boolean acusti_warn_b,
      in Boolean cC_active_b,
      in Boolean limiter_active_b,
      in Boolean opti_warn_b,
      out Double v_Vehicle_kmh,
      in Double cCSetValue_kmh,
      in Double limiterSetValue_kmh,
      in Double brakeForce_pc,
      in Double acceleration_pc,
      out Boolean parkingBrake_b,
      out Double brakeForce_pedal_pc,
      out Double acceleration_pedal_pc,
      out Boolean cruiseControl_b,
      out Boolean limiter_b,
      out Double leverUp_stat,
      out Double leverDown_stat,
      out Double v_Sign_kmh,
      out Boolean sign_b,
      out Double distance_stat,
      out Double v_Obj_rel_kmh;
    <<Type="Display">> component Acusti_warn_b {
      ports
        in Boolean acusti_warn_bIn1;
    }
    <<Type="Display">> component CCSetValue {
      ports
        in Double cCSetValue_kmhIn1;
    }
    <<Type="Display">> component CC_active_b {
      ports
        in Boolean cC_active_bIn1;
    }
    <<Type="Constant",Value="1">> component Constant {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant1 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant10 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="2">> component Constant11 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant12 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant2 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant3 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant4 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="400">> component Constant5 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant6 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="1">> component Constant7 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="0">> component Constant8 {
      ports
        out Double out1;
    }
    <<Type="Constant",Value="0">> component Constant9 {
      ports
        out Double out1;
    }
    <<Type="DataTypeConversion",OutDataTypeStr="int8">> component DataTypeConversion {
      ports
        in Double distance_statIn1,
        out Double distance_statOut1;
      effect distance_statIn1 -> distance_statOut1;
    }
    <<Type="Display">> component Distance_Object_m {
      ports
        in Double distance_Object_mIn1;
    }
    <<Type="SubSystem">> component FalseBlock {
      ports
        out Double yOut1;
      <<Type="Constant",Value="0">> component Zero {
        ports
          out Double out1;
      }
      component Zero zero;
      connect zero.out1 -> yOut1;
    }
    <<Type="SubSystem">> component FalseBlock1 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="0">> component Zero {
        ports
          out Double out1;
      }
      component Zero zero;
      connect zero.out1 -> yOut1;
    }
    <<Type="SubSystem">> component FalseBlock2 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="0">> component Zero {
        ports
          out Double out1;
      }
      component Zero zero;
      connect zero.out1 -> yOut1;
    }
    <<Type="SubSystem">> component FalseBlock5 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="0">> component Zero {
        ports
          out Double out1;
      }
      component Zero zero;
      connect zero.out1 -> yOut1;
    }
    <<Type="ManualSwitch">> component Lever_Level {
      ports
        in Double in1,
        in Double in2,
        out Double out1;
      effect in1 -> out1;
      effect in2 -> out1;
    }
    <<Type="Display">> component LimiterSetValue {
      ports
        in Double limiterSetValue_kmhIn1;
    }
    <<Type="Display">> component Limiter_active_b {
      ports
        in Boolean limiter_active_bIn1;
    }
    <<Type="ManualSwitch">> component ManualSwitch {
      ports
        in Double in1,
        in Double in2,
        out Boolean parkingBrake_bOut1;
      effect in1 -> parkingBrake_bOut1;
      effect in2 -> parkingBrake_bOut1;
    }
    <<Type="ManualSwitch">> component ManualSwitch1 {
      ports
        in Double in1,
        in Double in2,
        out Boolean sign_bOut1;
      effect in1 -> sign_bOut1;
      effect in2 -> sign_bOut1;
    }
    <<Type="ManualSwitch">> component ManualSwitch2 {
      ports
        in Double in1,
        in Double in2,
        out Boolean cruiseControl_bOut1;
      effect in1 -> cruiseControl_bOut1;
      effect in2 -> cruiseControl_bOut1;
    }
    <<Type="ManualSwitch">> component ManualSwitch5 {
      ports
        in Double in1,
        in Double in2,
        out Boolean limiter_bOut1;
      effect in1 -> limiter_bOut1;
      effect in2 -> limiter_bOut1;
    }
    <<Type="ManualSwitch">> component ManualSwitch6 {
      ports
        in Double distance_Object_Start_mIn1,
        in Double in2,
        out Double out1;
      effect distance_Object_Start_mIn1 -> out1;
      effect in2 -> out1;
    }
    <<Type="ManualSwitch">> component ManualSwitch7 {
      ports
        in Double in1,
        in Double in2,
        out Double out1;
      effect in1 -> out1;
      effect in2 -> out1;
    }
    <<Type="ManualSwitch">> component ManualSwitch8 {
      ports
        in Double in1,
        in Double in2,
        out Double out1;
      effect in1 -> out1;
      effect in2 -> out1;
    }
    <<Type="Product",Inputs="2">> component Mul {
      ports
        in Double in1,
        in Double in2,
        out Double leverUp_statOut1;
      effect in1 -> leverUp_statOut1;
      effect in2 -> leverUp_statOut1;
    }
    <<Type="Product",Inputs="2">> component Mul1 {
      ports
        in Double in1,
        in Double in2,
        out Double leverDown_statOut1;
      effect in1 -> leverDown_statOut1;
      effect in2 -> leverDown_statOut1;
    }
    <<Type="SubSystem">> component Object {
      ports
        in Double v_Object_kmhIn1,
        in Double distance_Object_Start_mIn2,
        in Double v_Vehicle_kmhIn3,
        out Double v_Obj_rel_kmhOut1,
        out Double distance_Object_mOut2;
      <<Type="SubSystem">> component Accumulator_REL {
        ports
          in Double mXIn1,
          in Double mNIn2,
          in Double uIn3,
          in Boolean eIn4,
          in Boolean rIn5,
          in Double iVIn6,
          out Double yOut1,
          out Boolean b_maxOut2,
          out Boolean b_minOut3;
        <<Type="SubSystem">> component Limit {
          ports
            in Double mXIn1,
            in Double mNIn2,
            in Double uIn3,
            out Double yOut1,
            out Boolean b_maxOut2,
            out Boolean b_minOut3;
          <<Operator="AND",Type="Logic">> component LogOp_A {
            ports
              in Boolean in1,
              in Boolean in2,
              out Boolean out1;
            effect in1 -> out1;
            effect in2 -> out1;
          }
          <<Operator="NOT",Type="Logic">> component LogOp_N {
            ports
              in Boolean in1,
              out Boolean out1;
            effect in1 -> out1;
          }
          <<Operator="<",Type="RelationalOperator">> component RelOp_MN {
            ports
              in Double in1,
              in Double in2,
              out Boolean out1;
            effect in1 -> out1;
            effect in2 -> out1;
          }
          <<Operator=">",Type="RelationalOperator">> component RelOp_MX {
            ports
              in Double in1,
              in Double in2,
              out Boolean out1;
            effect in1 -> out1;
            effect in2 -> out1;
          }
          <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_MN {
            ports
              in Double ifIn,
              in Boolean condition,
              in Double elseIn,
              out Double out1;
            effect ifIn -> out1;
            effect condition -> out1;
            effect elseIn -> out1;
          }
          <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_MX {
            ports
              in Double ifIn,
              in Boolean condition,
              in Double elseIn,
              out Double out1;
            effect ifIn -> out1;
            effect condition -> out1;
            effect elseIn -> out1;
          }
          <<Condition="u2 >= 1",Type="Condition">> component Condition {
            ports
              in Boolean in1,
              out Boolean out1;
          }
          <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
            ports
              in Boolean in1,
              out Boolean out1;
          }
          component LogOp_A logOp_A;
          component LogOp_N logOp_N;
          component RelOp_MN relOp_MN;
          component RelOp_MX relOp_MX;
          component Switch_MN switch_MN;
          component Switch_MX switch_MX;
          component Condition condition;
          component Condition1 condition1;
          connect condition.out1 -> switch_MX.condition;
          connect condition1.out1 -> switch_MN.condition;
          connect relOp_MX.out1 -> condition.in1;
          connect relOp_MN.out1 -> condition1.in1;
          connect relOp_MX.out1 -> b_maxOut2;
          connect relOp_MX.out1 -> logOp_N.in1;
          connect mXIn1 -> switch_MX.ifIn;
          connect mXIn1 -> relOp_MX.in2;
          connect mNIn2 -> relOp_MN.in2;
          connect mNIn2 -> switch_MN.ifIn;
          connect switch_MN.out1 -> switch_MX.elseIn;
          connect logOp_N.out1 -> logOp_A.in1;
          connect logOp_A.out1 -> b_minOut3;
          connect relOp_MN.out1 -> logOp_A.in2;
          connect uIn3 -> switch_MN.elseIn;
          connect uIn3 -> relOp_MX.in1;
          connect uIn3 -> relOp_MN.in1;
          connect switch_MX.out1 -> yOut1;
        }
        <<Type="UnitDelay",InitialCondition="0">> component Memory_Y {
          ports
            in Double in1,
            out Double out1;
          effect in1 -> out1;
        }
        <<Type="Sum",ListOfSigns="++">> component Sum_yu {
          ports
            in Double in1,
            in Double in2,
            out Double out1;
          effect in1 -> out1;
          effect in2 -> out1;
        }
        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_E {
          ports
            in Double ifIn,
            in Boolean condition,
            in Double elseIn,
            out Double out1;
          effect ifIn -> out1;
          effect condition -> out1;
          effect elseIn -> out1;
        }
        <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component Switch_R {
          ports
            in Double ifIn,
            in Boolean condition,
            in Double elseIn,
            out Double out1;
          effect ifIn -> out1;
          effect condition -> out1;
          effect elseIn -> out1;
        }
        <<Condition="u2 >= 1",Type="Condition">> component Condition {
          ports
            in Boolean in1,
            out Boolean out1;
        }
        <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
          ports
            in Boolean in1,
            out Boolean out1;
        }
        component Limit limit;
        component Memory_Y memory_Y;
        component Sum_yu sum_yu;
        component Switch_E switch_E;
        component Switch_R switch_R;
        component Condition condition;
        component Condition1 condition1;
        connect condition1.out1 -> switch_E.condition;
        connect condition.out1 -> switch_R.condition;
        connect rIn5 -> condition.in1;
        connect eIn4 -> condition1.in1;
        connect switch_R.out1 -> limit.uIn3;
        connect mXIn1 -> limit.mXIn1;
        connect mNIn2 -> limit.mNIn2;
        connect limit.b_minOut3 -> b_minOut3;
        connect limit.b_maxOut2 -> b_maxOut2;
        connect limit.yOut1 -> memory_Y.in1;
        connect limit.yOut1 -> yOut1;
        connect memory_Y.out1 -> switch_E.elseIn;
        connect memory_Y.out1 -> sum_yu.in2;
        connect iVIn6 -> switch_R.ifIn;
        connect switch_E.out1 -> switch_R.elseIn;
        connect sum_yu.out1 -> switch_E.ifIn;
        connect uIn3 -> sum_yu.in1;
      }
      <<Type="Sum",ListOfSigns="+-">> component Add {
        ports
          in Double in1,
          in Double in2,
          out Double v_Obj_rel_kmhOut1;
        effect in1 -> v_Obj_rel_kmhOut1;
        effect in2 -> v_Obj_rel_kmhOut1;
      }
      <<Type="Constant",Value="300">> component Constant {
        ports
          out Double out1;
      }
      <<Type="Constant",Value="1000">> component Constant1 {
        ports
          out Double out1;
      }
      <<Type="Constant",Value="0">> component Constant2 {
        ports
          out Double out1;
      }
      <<Type="Gain",Gain="-1/3.6">> component Gain {
        ports
          in Double v_Obj_rel_kmhIn1,
          out Double out1;
        effect v_Obj_rel_kmhIn1 -> out1;
      }
      <<Operator="NOT",Type="Logic">> component LogicalOperator {
        ports
          in Boolean in1,
          out Boolean out1;
        effect in1 -> out1;
      }
      <<Operator=">",Type="RelationalOperator">> component RelOp {
        ports
          in Double in1,
          in Double in2,
          out Boolean out1;
        effect in1 -> out1;
        effect in2 -> out1;
      }
      <<Type="Terminator">> component Terminator {
        ports
          in Boolean in1;
      }
      <<Type="Terminator">> component Terminator1 {
        ports
          in Boolean in1;
      }
      component Accumulator_REL accumulator_REL;
      component Add add;
      component Constant constant;
      component Constant1 constant1;
      component Constant2 constant2;
      component Gain gain;
      component LogicalOperator logicalOperator;
      component RelOp relOp;
      component Terminator terminator;
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
    <<Type="Display">> component Opti_warn_b {
      ports
        in Boolean opti_warn_bIn1;
    }
    <<Type="SubSystem">> component SliderGain {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component SliderGain1 {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component SliderGain2 {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component SliderGain3 {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component SliderGain4 {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component SliderGain5 {
      ports
        in Double uIn1,
        out Double yOut1;
      <<Type="Gain",Gain="gain">> component SliderGain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      component SliderGain sliderGain;
      connect sliderGain.out1 -> yOut1;
      connect uIn1 -> sliderGain.in1;
    }
    <<Type="SubSystem">> component TrueBlock {
      ports
        out Double yOut1;
      <<Type="Constant",Value="1">> component One {
        ports
          out Double out1;
      }
      component One one;
      connect one.out1 -> yOut1;
    }
    <<Type="SubSystem">> component TrueBlock1 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="1">> component One {
        ports
          out Double out1;
      }
      component One one;
      connect one.out1 -> yOut1;
    }
    <<Type="SubSystem">> component TrueBlock2 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="1">> component One {
        ports
          out Double out1;
      }
      component One one;
      connect one.out1 -> yOut1;
    }
    <<Type="SubSystem">> component TrueBlock5 {
      ports
        out Double yOut1;
      <<Type="Constant",Value="1">> component One {
        ports
          out Double out1;
      }
      component One one;
      connect one.out1 -> yOut1;
    }
    <<Type="Display">> component V_Vehicle_kmh {
      ports
        in Double v_Vehicle_kmhIn1;
    }
    <<Type="SubSystem">> component Vehicle {
      ports
        out Double v_Vehicle_kmhOut1,
        in Double brakeForce_pc,
        in Double acceleration_pc;
      <<Type="Constant",Value="0">> component Constant {
        ports
          out Double out1;
      }
      <<Type="Constant",Value="0">> component Constant1 {
        ports
          out Double out1;
      }
      <<Type="SubSystem">> component DiscreteTransferFcnwithinitialstates {
        ports
          in Double in1In1,
          out Double out1Out1;
        <<Type="DiscreteStateSpace">> component DiscreteStateSpace {
          ports
            in Double in1,
            out Double out1;
          effect in1 -> out1;
        }
        component DiscreteStateSpace discreteStateSpace;
        connect discreteStateSpace.out1 -> out1Out1;
        connect in1In1 -> discreteStateSpace.in1;
      }
      <<Type="Gain",Gain="-2">> component Gain {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      <<Operator="<",Type="RelationalOperator">> component RelationalOperator {
        ports
          in Double in1,
          in Double in2,
          out Boolean out1;
        effect in1 -> out1;
        effect in2 -> out1;
      }
      <<Type="Saturate",LowerLimit="0",UpperLimit="250">> component Saturation {
        ports
          in Double in1,
          out Double v_Vehicle_kmhOut1;
        effect in1 -> v_Vehicle_kmhOut1;
      }
      <<Type="Sum",ListOfSigns="+--">> component Sum1 {
        ports
          in Double v_Vehicle_kmhIn1,
          in Double in2,
          in Double in3,
          out Double out1;
        effect v_Vehicle_kmhIn1 -> out1;
        effect in2 -> out1;
        effect in3 -> out1;
      }
      <<Type="Switch",Criteria="u2 > Threshold",Threshold="0">> component SwitchBlock {
        ports
          in Double ifIn,
          in Boolean condition,
          in Double elseIn,
          out Double out1;
        effect ifIn -> out1;
        effect condition -> out1;
        effect elseIn -> out1;
      }
      <<Type="Switch",Criteria="u2 >= Threshold",Threshold="1">> component SwitchBlock1 {
        ports
          in Double ifIn,
          in Boolean condition,
          in Double elseIn,
          out Double out1;
        effect ifIn -> out1;
        effect condition -> out1;
        effect elseIn -> out1;
      }
      <<Type="UniformRandomNumber">> component UniformRandomNumber {
        ports
          out Double out1;
      }
      <<Type="UnitDelay",InitialCondition="0">> component UnitDelay {
        ports
          in Double v_Vehicle_kmhIn1,
          out Double out1;
        effect v_Vehicle_kmhIn1 -> out1;
      }
      <<Type="Lookup">> component Widerstand {
        ports
          in Double in1,
          out Double out1;
        effect in1 -> out1;
      }
      <<Condition="u2 > 0",Type="Condition">> component Condition {
        ports
          in Double in1,
          out Boolean out1;
      }
      <<Condition="u2 >= 1",Type="Condition">> component Condition1 {
        ports
          in Boolean in1,
          out Boolean out1;
      }
      component Constant constant;
      component Constant1 constant1;
      component DiscreteTransferFcnwithinitialstates discreteTransferFcnwithinitialstates;
      component Gain gain;
      component RelationalOperator relationalOperator;
      component Saturation saturation;
      component Sum1 sum1;
      component SwitchBlock switchBlock;
      component SwitchBlock1 switchBlock1;
      component UniformRandomNumber uniformRandomNumber;
      component UnitDelay unitDelay;
      component Widerstand widerstand;
      component Condition condition;
      component Condition1 condition1;
      connect condition.out1 -> switchBlock.condition;
      connect brakeForce_pc -> switchBlock1.ifIn;
      connect condition1.out1 -> switchBlock1.condition;
      connect acceleration_pc -> switchBlock.elseIn;
      connect switchBlock1.out1 -> condition.in1;
      connect relationalOperator.out1 -> condition1.in1;
      connect constant1.out1 -> switchBlock1.elseIn;
      connect switchBlock1.out1 -> gain.in1;
      connect constant.out1 -> relationalOperator.in1;
      connect uniformRandomNumber.out1 -> sum1.in3;
      connect unitDelay.out1 -> widerstand.in1;
      connect unitDelay.out1 -> relationalOperator.in2;
      connect widerstand.out1 -> sum1.in2;
      connect discreteTransferFcnwithinitialstates.out1Out1 -> sum1.v_Vehicle_kmhIn1;
      connect sum1.out1 -> saturation.in1;
      connect switchBlock.out1 -> discreteTransferFcnwithinitialstates.in1In1;
      connect gain.out1 -> switchBlock.ifIn;
      connect saturation.v_Vehicle_kmhOut1 -> v_Vehicle_kmhOut1;
      connect saturation.v_Vehicle_kmhOut1 -> unitDelay.v_Vehicle_kmhIn1;
    }
    component Acusti_warn_b acusti_warn_b;
    component CCSetValue cCSetValue;
    component CC_active_b cC_active_b;
    component Constant constant;
    component Constant1 constant1;
    component Constant10 constant10;
    component Constant11 constant11;
    component Constant12 constant12;
    component Constant2 constant2;
    component Constant3 constant3;
    component Constant4 constant4;
    component Constant5 constant5;
    component Constant6 constant6;
    component Constant7 constant7;
    component Constant8 constant8;
    component Constant9 constant9;
    component DataTypeConversion dataTypeConversion;
    component Distance_Object_m distance_Object_m;
    component FalseBlock falseBlock;
    component FalseBlock1 falseBlock1;
    component FalseBlock2 falseBlock2;
    component FalseBlock5 falseBlock5;
    component Lever_Level lever_Level;
    component LimiterSetValue limiterSetValue;
    component Limiter_active_b limiter_active_b;
    component ManualSwitch manualSwitch;
    component ManualSwitch1 manualSwitch1;
    component ManualSwitch2 manualSwitch2;
    component ManualSwitch5 manualSwitch5;
    component ManualSwitch6 manualSwitch6;
    component ManualSwitch7 manualSwitch7;
    component ManualSwitch8 manualSwitch8;
    component Mul mul;
    component Mul1 mul1;
    component Object object;
    component Opti_warn_b opti_warn_b;
    component SliderGain sliderGain;
    component SliderGain1 sliderGain1;
    component SliderGain2 sliderGain2;
    component SliderGain3 sliderGain3;
    component SliderGain4 sliderGain4;
    component SliderGain5 sliderGain5;
    component TrueBlock trueBlock;
    component TrueBlock1 trueBlock1;
    component TrueBlock2 trueBlock2;
    component TrueBlock5 trueBlock5;
    component V_Vehicle_kmh v_Vehicle_kmh;
    component Vehicle vehicle;
    connect vehicle.v_Vehicle_kmhOut1 -> v_Vehicle_kmh;
    connect acusti_warn_b -> acusti_warn_b.acusti_warn_bIn1;
    connect opti_warn_b -> opti_warn_b.opti_warn_bIn1;
    connect limiter_active_b -> limiter_active_b.limiter_active_bIn1;
    connect limiterSetValue_kmh -> limiterSetValue.limiterSetValue_kmhIn1;
    connect cCSetValue_kmh -> cCSetValue.cCSetValue_kmhIn1;
    connect cC_active_b -> cC_active_b.cC_active_bIn1;
    connect brakeForce_pc -> vehicle.brakeForce_pc;
    connect acceleration_pc -> vehicle.acceleration_pc;
    connect manualSwitch.parkingBrake_bOut1 -> parkingBrake_b;
    connect sliderGain1.yOut1 -> brakeForce_pedal_pc;
    connect sliderGain.yOut1 -> acceleration_pedal_pc;
    connect manualSwitch2.cruiseControl_bOut1 -> cruiseControl_b;
    connect manualSwitch5.limiter_bOut1 -> limiter_b;
    connect mul.leverUp_statOut1 -> leverUp_stat;
    connect mul1.leverDown_statOut1 -> leverDown_stat;
    connect sliderGain2.yOut1 -> v_Sign_kmh;
    connect manualSwitch1.sign_bOut1 -> sign_b;
    connect dataTypeConversion.distance_statOut1 -> distance_stat;
    connect object.v_Obj_rel_kmhOut1 -> v_Obj_rel_kmh;
    connect constant12.out1 -> sliderGain5.uIn1;
    connect sliderGain5.yOut1 -> dataTypeConversion.distance_statIn1;
    connect manualSwitch8.out1 -> mul1.in1;
    connect lever_Level.out1 -> mul1.in2;
    connect lever_Level.out1 -> mul.in2;
    connect constant11.out1 -> lever_Level.in2;
    connect constant10.out1 -> lever_Level.in1;
    connect manualSwitch7.out1 -> mul.in1;
    connect constant9.out1 -> manualSwitch8.in2;
    connect constant8.out1 -> manualSwitch7.in2;
    connect constant7.out1 -> manualSwitch8.in1;
    connect constant6.out1 -> manualSwitch7.in1;
    connect object.distance_Object_mOut2 -> distance_Object_m.distance_Object_mIn1;
    connect manualSwitch6.out1 -> object.distance_Object_Start_mIn2;
    connect constant5.out1 -> manualSwitch6.in2;
    connect sliderGain4.yOut1 -> manualSwitch6.distance_Object_Start_mIn1;
    connect constant4.out1 -> sliderGain4.uIn1;
    connect sliderGain3.yOut1 -> object.v_Object_kmhIn1;
    connect constant3.out1 -> sliderGain3.uIn1;
    connect constant2.out1 -> sliderGain2.uIn1;
    connect falseBlock1.yOut1 -> manualSwitch1.in2;
    connect trueBlock1.yOut1 -> manualSwitch1.in1;
    connect constant1.out1 -> sliderGain1.uIn1;
    connect constant.out1 -> sliderGain.uIn1;
    connect vehicle.v_Vehicle_kmhOut1 -> v_Vehicle_kmh.v_Vehicle_kmhIn1;
    connect vehicle.v_Vehicle_kmhOut1 -> object.v_Vehicle_kmhIn3;
    connect falseBlock5.yOut1 -> manualSwitch5.in2;
    connect trueBlock5.yOut1 -> manualSwitch5.in1;
    connect falseBlock2.yOut1 -> manualSwitch2.in2;
    connect trueBlock2.yOut1 -> manualSwitch2.in1;
    connect falseBlock.yOut1 -> manualSwitch.in2;
    connect trueBlock.yOut1 -> manualSwitch.in1;
  }
  <<Type="Memory">> component Memory1 {
    ports
      in Boolean in1,
      out Boolean out1;
  }
  <<Type="Memory">> component Memory2 {
    ports
      in Double in1,
      out Double out1;
  }
  <<Type="Memory">> component Memory3 {
    ports
      in Double in1,
      out Double out1;
  }
  <<Type="Memory">> component Memory4 {
    ports
      in Boolean in1,
      out Boolean out1;
  }
  <<Type="Memory">> component Memory5 {
    ports
      in Boolean in1,
      out Boolean out1;
  }
  <<Type="Memory">> component Memory6 {
    ports
      in Boolean in1,
      out Boolean out1;
  }
  <<Type="Memory">> component Memory7 {
    ports
      in Double in1,
      out Double out1;
  }
  <<Type="Memory">> component Memory8 {
    ports
      in Double in1,
      out Double out1;
  }
  component DEMO_FAS dEMO_FAS;
  component SimToRealTime simToRealTime;
  component Umgebung umgebung;
  component Memory1 memory1;
  component Memory2 memory2;
  component Memory3 memory3;
  component Memory4 memory4;
  component Memory5 memory5;
  component Memory6 memory6;
  component Memory7 memory7;
  component Memory8 memory8;
  connect memory1.out1 -> umgebung.cC_active_b;
  connect memory2.out1 -> umgebung.cCSetValue_kmh;
  connect memory3.out1 -> umgebung.limiterSetValue_kmh;
  connect memory4.out1 -> umgebung.limiter_active_b;
  connect memory5.out1 -> umgebung.opti_warn_b;
  connect memory6.out1 -> umgebung.acusti_warn_b;
  connect memory7.out1 -> umgebung.brakeForce_pc;
  connect memory8.out1 -> umgebung.acceleration_pc;
  connect dEMO_FAS.cC_active_b -> memory1.in1;
  connect dEMO_FAS.cCSetValue_kmh -> memory2.in1;
  connect dEMO_FAS.limiterSetValue_kmh -> memory3.in1;
  connect dEMO_FAS.limiter_active_b -> memory4.in1;
  connect dEMO_FAS.opti_warn_b -> memory5.in1;
  connect dEMO_FAS.acusti_warn_b -> memory6.in1;
  connect dEMO_FAS.brakeForce_pc -> memory7.in1;
  connect dEMO_FAS.acceleration_pc -> memory8.in1;
  connect umgebung.parkingBrake_b -> dEMO_FAS.parkingBrake_b;
  connect umgebung.brakeForce_pedal_pc -> dEMO_FAS.brakeForce_pedal_pc;
  connect umgebung.acceleration_pedal_pc -> dEMO_FAS.acceleration_pedal_pc;
  connect umgebung.cruiseControl_b -> dEMO_FAS.cruiseControl_b;
  connect umgebung.limiter_b -> dEMO_FAS.limiter_b;
  connect umgebung.leverUp_stat -> dEMO_FAS.leverUp_stat;
  connect umgebung.leverDown_stat -> dEMO_FAS.leverDown_stat;
  connect umgebung.v_Vehicle_kmh -> dEMO_FAS.v_Vehicle_kmh;
  connect umgebung.v_Sign_kmh -> dEMO_FAS.v_Sign_kmh;
  connect umgebung.sign_b -> dEMO_FAS.sign_b;
  connect umgebung.distance_stat -> dEMO_FAS.distance_stat;
  connect umgebung.v_Obj_rel_kmh -> dEMO_FAS.v_Obj_rel_kmh;
}
