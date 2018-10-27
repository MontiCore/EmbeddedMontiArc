package de.monticore.lang.monticar.generator.cpp.converter;

import alice.tuprolog.Var;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.VariablePortValueChecker;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.ExecuteDynamicConnects;
import de.se_rwth.commons.logging.Log;

import javax.swing.plaf.metal.MetalTheme;
import java.util.*;

public class EventConverter {

    public static void generateEvents(Method executeMethod, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings){
        if(!(componentSymbol instanceof EMADynamicComponentInstanceSymbol)){
            return;
        }
        EMADynamicComponentInstanceSymbol dynComponent = (EMADynamicComponentInstanceSymbol) componentSymbol;


        for(EMADynamicEventHandlerInstanceSymbol event : dynComponent.getEventHandlers()){
            Log.info("Create Event: "+event.getName(), "EventConverter");

            boolean generateCondition = false;

            if(event.isDynamicPortConnectionEvent()) {
//                generateCondition = generateDynamicConnectEvent(event, componentSymbol, executeMethod, bluePrint);
                int x = 5/0;
                generateCondition = EventDynamicConnectConverter.generateDynamicConnectEvent(event, componentSymbol, executeMethod, bluePrint);
            }else{
                Log.info("Create connectors for: "+event.getFullName(), "EventConverter");

                generateCondition = generateValueEvent(event, executeMethod, bluePrint);
            }


            if (generateCondition) {
                //generate event condition method
                generateEventConditionMethod(event, componentSymbol, bluePrint);
            }
        }
    }

    protected static void generateEventConditionMethod(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol,BluePrintCPP bluePrint ){
        Method condition = new Method(EventConnectInstructionCPP.getEventNameCPP(event.getFullName()), "bool");
        condition.setPublic(false);

        String conditionExpression = "return ";
        conditionExpression += generateEventCondition(event.getCondition(), componentSymbol, bluePrint);
        conditionExpression += ";\n";


        condition.addInstruction(new TargetCodeInstruction(conditionExpression));
        bluePrint.addMethod(condition);
    }

    protected static boolean generateValueEvent(EMADynamicEventHandlerInstanceSymbol event, Method executeMethod, BluePrintCPP bluePrint ){
        int number = 0;
        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
            if(connector.isDynamicSourceNewComponent() || connector.isDynamicSourceNewPort() ||
                    connector.isDynamicTargetNewComponent() || connector.isDynamicTargetNewPort()){
                continue;
            }

            generateEventConnector(event.getFullName(), connector, bluePrint, executeMethod);
            ++number;
        }
        return number > 0;
    }

    @Deprecated
    protected static boolean generateDynamicConnectEvent(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, Method executeMethod, BluePrintCPP bluePrint ) {

//        EMADynamicComponentInstanceSymbol dynComp = (EMADynamicComponentInstanceSymbol)componentSymbol;
//
//        for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
//            EMADynamicConnectorInstanceSymbol dynConnect = (EMADynamicConnectorInstanceSymbol) connector;
//
////            body.addInstruction(new TargetCodeInstruction(
////                    "// connect: "+dynConnect.getSource()+" -> "+dynConnect.getTarget()+"\n"
////            ));
//
//            String afterComponent = "";
//            String sourceName = dynConnect.getSource();
//            String targetName = dynConnect.getTarget();
//            EMAPortInstanceSymbol target = dynConnect.getTargetPort();
//
//            if(dynConnect.isDynamicSourceNewPort()){
//                if(sourceName.contains(".")){
//                    //TODO target hat eine komponente
//                }else{
//                    sourceName = EMAPortSymbol.getNameWithoutArrayBracketPart(sourceName);
//                    sourceName = String.format("%s[_%s_dynPortID]", sourceName, sourceName);
//                }
//            }
//
//            if(dynConnect.isDynamicTargetNewPort()){
//                if(targetName.contains(".")){
//                    //TODO target hat eine komponente
//                    System.out.println("bla bla");
//                }else{
//                    targetName = EMAPortSymbol.getNameWithoutArrayBracketPart(targetName);
//                    targetName = String.format("%s[_%s_dynPortID]", targetName, targetName);
//                }
//            }
//
//            Optional<VariableType> vt = TypeConverter.getVariableTypeForMontiCarTypeName(target.getTypeReference().getName());
//            generateEventDynamicConnectVecotr(vt.get().getTypeNameTargetLanguage(), bluePrint);
//
//
//            body.addInstruction(new TargetCodeInstruction(String.format(
////                    "// connect: "+sourceName+" - "+targetName+"\n"
//                    "__dynamic_%s_connect.push_back({%s, &%s, &%s});\n", vt.get().getTypeNameTargetLanguage(), afterComponent, sourceName, targetName
//            )));
//
////            executeMethod.addInstruction(new TargetCodeInstruction(
////                    "executeDynamicConnects("+afterComponent+");\n"
////            ));
//            executeMethod.addInstruction(new ExecuteDynamicConnects(afterComponent));
//        }
//


        return true;
    }



    public static void generatePVCNextMethod(BluePrintCPP bluePrint){
        Method next = new Method("next", "void");
        next.setPublic(false);

        for(Variable v : bluePrint.getVariables()){
            if(v instanceof VariablePortValueChecker){
                next.addInstruction(((VariablePortValueChecker) v).nextValueInstruction());
            }
        }

        if(!next.getInstructions().isEmpty()){
            bluePrint.addMethod(next);
        }
    }

    protected static void generateEventConnector(String eventName, EMADynamicConnectorInstanceSymbol connector, BluePrintCPP bluePrint, Method method){
        if (!connector.isConstant()) {
            Log.info("source:" + connector.getSource() + " target:" + connector.getTarget(), "Port info:");
            Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);
            Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);
            Log.info("v1: " + v1.getName() + " v2: " + v2.getName(), "Variable Info:");
            Log.info("v1: " + v1.getNameTargetLanguageFormat() + " v2: " + v2.getNameTargetLanguageFormat(), "Variable Info:");

            Instruction instruction = new EventConnectInstructionCPP(eventName, v2, v1);
            method.addInstruction(instruction);
        } else {
            if (connector.getSourcePort().isConstant()) {
                EMAPortInstanceSymbol constPort =  connector.getSourcePort();
                Variable v1 = new Variable();
                v1.setName(constPort.getConstantValue().get().getValueAsString());
                Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);


                Instruction instruction = new EventConnectInstructionCPP(eventName, v2, v1);
                method.addInstruction(instruction);
            } else if (connector.getTargetPort().isConstant()) {
                EMAPortInstanceSymbol constPort = connector.getTargetPort();
                Variable v2 = new Variable();
                v2.setName(constPort.getConstantValue().get().getValueAsString());
                Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);


                Instruction instruction = new EventConnectInstructionCPP(eventName, v2, v1);
                method.addInstruction(instruction);
            } else {
                Log.error("0xWRONGCONNECTOR the connector is constant but target nor source are constant");
            }
        }
    }





    //<editor-fold desc="Generate event condition">

    protected static String generateEventCondition(EventExpressionSymbol expression, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){
        if(expression instanceof EventBracketExpressionSymbol){
            return "( "+generateEventCondition(((EventBracketExpressionSymbol) expression).getInnerExpression(), componentSymbol, bluePrint)+" )";
        }else if(expression instanceof EventLogicalOperationExpressionSymbol){
            return generateEventConditionEventLogicalExpressionSymbol((EventLogicalOperationExpressionSymbol) expression, componentSymbol, bluePrint);
        }else if(expression instanceof EventPortExpressionValueSymbol){
            return generateEventConditionEventPortValueSymbol((EventPortExpressionValueSymbol) expression, componentSymbol, bluePrint);
        }else if(expression instanceof EventPortExpressionConnectSymbol){
            return generateEventConditionEventPortConnectSymbol((EventPortExpressionConnectSymbol) expression, componentSymbol, bluePrint);
        }

        return "";
    }

    protected static String generateEventConditionEventLogicalExpressionSymbol(EventLogicalOperationExpressionSymbol expression, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){
        String result = "( ";
        if (expression.getLeftExpression() != null){
            result += generateEventCondition(expression.getLeftExpression(), componentSymbol, bluePrint);
        }

        result += expression.getOperator();

        if (expression.getRightExpression() != null){
            result += generateEventCondition(expression.getRightExpression(), componentSymbol, bluePrint);
        }

        return  result+" )";
    }

    protected static String generateEventConditionEventPortValueSymbol(EventPortExpressionValueSymbol expressionValueSymbol, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){

        VariablePortValueChecker pvc = new VariablePortValueChecker(expressionValueSymbol.getName());
        bluePrint.addVariable(pvc);

        Optional<EMAPortInstanceSymbol> portSymbol = componentSymbol.getPortInstance(expressionValueSymbol.getName());
        if(portSymbol.isPresent()){
            String typeNameMontiCar = portSymbol.get().getTypeReference().getName();
            pvc.setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(typeNameMontiCar, pvc, portSymbol.get()).get());
        }

        addTest(expressionValueSymbol.getPortValue(), pvc);

        return pvc.getNameTargetLanguageFormat()+".check()";

    }

    protected static String generateEventConditionEventPortConnectSymbol(EventPortExpressionConnectSymbol expressionConnectSymbol, EMAComponentInstanceSymbol componentSymbol, BluePrint bluePrint){
        return "(!__"+expressionConnectSymbol.getName()+"_connect_request.empty())";
    }

    //</editor-fold>

    //<editor-fold desc="Generate concrete test for port values">

    public static void addTest(PortValueSymbol pvs, VariablePortValueChecker vpvc){
        if(pvs instanceof PortValueInputSymbol){
//            addTestPortValueInputSymbol((PortValueInputSymbol) pvs, vpvc);

            vpvc.addTestSymbol_Equals(((PortValueInputSymbol) pvs).getValueStringRepresentation());

        }else if(pvs instanceof PortValuesArraySymbol){
            PortValuesArraySymbol ar = (PortValuesArraySymbol)pvs;
            for(int i = 0; i < ar.size(); ++i){
                addTest(ar.getForIndex(i), vpvc);
            }
        }else if(pvs instanceof PortValuePrecisionSymbol){

            PortValuePrecisionSymbol ps = (PortValuePrecisionSymbol)pvs;
            String value = ps.getValue().getValueStringRepresentation();
            String prec = ps.getPrecision().getValueStringRepresentation();

            vpvc.addTestSymbol_Range("("+value+" - "+prec+")", "("+value+" + "+prec+")");
        }else if(pvs instanceof PortValueRangeSymbol){
            PortValueRangeSymbol ps = (PortValueRangeSymbol)pvs;
            String lower = ps.getLowerBound().getValueStringRepresentation();
            String upper = ps.getUpperBound().getValueStringRepresentation();

            vpvc.addTestSymbol_Range(lower, upper);
        }else if(pvs instanceof PortValueCompareSymbol){
            PortValueCompareSymbol compare = (PortValueCompareSymbol)pvs;

            vpvc.addTestSymbol_Compare(compare.getOperator(), compare.getCompareValue().getValueStringRepresentation());

        }
    }


    //</editor-fold>
}
