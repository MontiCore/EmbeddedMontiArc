package de.monticore.lang.monticar.generator.cpp.converter;

import alice.tuprolog.Var;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventBracketExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventLogicalOperationExpressionSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.EventPortExpressionValueSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueInputSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValueSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.PortValuesArraySymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.VariablePortValueChecker;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class EventConverter {

    public static void generateEventConnectors(Method method, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings){
        if(!(componentSymbol instanceof EMADynamicComponentInstanceSymbol)){
            return;
        }
        EMADynamicComponentInstanceSymbol dynComponent = (EMADynamicComponentInstanceSymbol) componentSymbol;


        for(EMADynamicEventHandlerInstanceSymbol event : dynComponent.getEventHandlers()){
            Log.info("Create connectors for: "+event.getFullName(), "EventConverter");

            int number = 0;
            for(EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()){
                if(connector.isDynamicSourceNewComponent() || connector.isDynamicSourceNewPort() || connector.isDynamicTargetNewComponent() || connector.isDynamicTargetNewPort()){
                    continue;
                }

                generateEventConnector(event.getFullName(), connector, bluePrint, method);
                ++number;
            }
            if (number > 0) {
                //generate event condition method
                Method condition = new Method(EventConnectInstructionCPP.getEventNameCPP(event.getFullName()), "bool");
                condition.setPublic(false);

                String conditionExpression = "return ";
                conditionExpression += generateEventCondition(event.getCondition(), componentSymbol, bluePrint);
                conditionExpression += ";\n";


                condition.addInstruction(new TargetCodeInstruction(conditionExpression));
                bluePrint.addMethod(condition);
            }
        }
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

    protected static String generateEventCondition(EventExpressionSymbol expression, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint){
        if(expression instanceof EventBracketExpressionSymbol){
            return "( "+generateEventCondition(((EventBracketExpressionSymbol) expression).getInnerExpression(), componentSymbol, bluePrint)+" )";
        }else if(expression instanceof EventLogicalOperationExpressionSymbol){
            return generateEventConditionEventLogicalExpressionSymbol((EventLogicalOperationExpressionSymbol) expression, componentSymbol, bluePrint);
        }else if(expression instanceof EventPortExpressionValueSymbol){
            return generateEventConditionEventPortValueSymbol((EventPortExpressionValueSymbol) expression, componentSymbol, bluePrint);
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

//        Variable v = new Variable()
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

    //<editor-fold desc="Generate concrete test for port values">

    public static void addTest(PortValueSymbol pvs, VariablePortValueChecker vpvc){
        if(pvs instanceof PortValueInputSymbol){
            addTestPortValueInputSymbol((PortValueInputSymbol) pvs, vpvc);
        }else if(pvs instanceof PortValuesArraySymbol){
            PortValuesArraySymbol ar = (PortValuesArraySymbol)pvs;
            for(int i = 0; i < ar.size(); ++i){
                addTest(ar.getForIndex(i), vpvc);
            }
        }
    }


    public static void addTestPortValueInputSymbol(PortValueInputSymbol pvis, VariablePortValueChecker vpvc){
        if(pvis.isLogic()){
            vpvc.addTestSymbol_Equals(pvis.isLogicValue() ? "true" : "false");
        }
    }



    //</editor-fold>
}
