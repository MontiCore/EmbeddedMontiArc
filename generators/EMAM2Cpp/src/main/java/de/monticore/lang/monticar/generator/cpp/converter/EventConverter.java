/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicEventHandlerInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.*;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.expression.portvalueexpressionvalues.*;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.EMAMBluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.VariablePortValueChecker;
import de.monticore.lang.monticar.generator.cpp.instruction.EventConnectInstructionCPP;
import de.se_rwth.commons.logging.Log;

import java.util.*;

public class EventConverter {

    public static void generateEvents(Method executeMethod, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        if (!(componentSymbol instanceof EMADynamicComponentInstanceSymbol)) {
            return;
        }
        EMADynamicComponentInstanceSymbol dynComponent = (EMADynamicComponentInstanceSymbol) componentSymbol;

        EventDynamicConnectConverter.resetFreePositionInCodeCounter();
        for (EMADynamicEventHandlerInstanceSymbol event : dynComponent.getEventHandlers()) {
            Log.info("Create Event: " + event.getName(), "EventConverter");

            boolean generateCondition = false;

            if (event.isDynamicPortConnectionEvent()) {
//                generateCondition = generateDynamicConnectEvent(event, componentSymbol, executeMethod, bluePrint);
                generateCondition = EventDynamicConnectConverter.generateDynamicConnectEvent(event, componentSymbol, executeMethod, bluePrint);
            } else if (event.isDynamicPortFreeEvent()) {
//                generateCondition = generateFreeEvent(event, executeMethod, bluePrint);
                generateCondition = EventDynamicConnectConverter.free_generateDynamicFreeEvent(event, componentSymbol, executeMethod, bluePrint);
            } else {
                Log.info("Create connectors for: " + event.getFullName(), "EventConverter");

                generateCondition = generateValueEvent(event, executeMethod, bluePrint);
            }


            if (generateCondition) {
                //generate event condition method
                generateEventConditionMethod(event, componentSymbol, bluePrint);
            }
        }
    }

    protected static void generateEventConditionMethod(EMADynamicEventHandlerInstanceSymbol event, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint) {
        Method condition = new Method(EventConnectInstructionCPP.getEventNameCPP(event.getName()), "bool");
        condition.setPublic(false);

        String conditionExpression = "return ";
        conditionExpression += generateEventCondition(event.getCondition(), componentSymbol, bluePrint);
        conditionExpression += ";\n";


        condition.addInstruction(new TargetCodeInstruction(conditionExpression));
        bluePrint.addMethod(condition);
    }

    protected static boolean generateValueEvent(EMADynamicEventHandlerInstanceSymbol event, Method executeMethod, EMAMBluePrintCPP bluePrint) {
        int number = 0;
        for (EMADynamicConnectorInstanceSymbol connector : event.getConnectorsDynamic()) {
            if (connector.isDynamicSourceNewComponent() || connector.isDynamicSourceNewPort() ||
                    connector.isDynamicTargetNewComponent() || connector.isDynamicTargetNewPort()) {
                continue;
            }

            generateEventConnector(event.getFullName(), connector, bluePrint, executeMethod);
            ++number;
        }
        return number > 0;
    }

    protected static boolean generateFreeEvent(EMADynamicEventHandlerInstanceSymbol event, Method executeMethod, EMAMBluePrint bluePrint) {


        return true;
    }

    public static void generatePVCNextMethod(EMAMBluePrintCPP bluePrint) {
        Method next = new Method("next", "void");
        next.setPublic(false);

        for (Variable v : bluePrint.getVariables()) {
            if (v instanceof VariablePortValueChecker) {
                next.addInstruction(((VariablePortValueChecker) v).nextValueInstruction());
            }
        }

        if (!next.getInstructions().isEmpty()) {
            bluePrint.addMethod(next);
        }
    }

    protected static void generateEventConnector(String eventName, EMADynamicConnectorInstanceSymbol connector, EMAMBluePrintCPP bluePrint, Method method) {
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
                EMAPortInstanceSymbol constPort = connector.getSourcePort();
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

    protected static String generateEventCondition(EventExpressionSymbol expression, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint) {
        if (expression instanceof EventBracketExpressionSymbol) {
            return "( " + generateEventCondition(((EventBracketExpressionSymbol) expression).getInnerExpression(), componentSymbol, bluePrint) + " )";
        } else if (expression instanceof EventLogicalOperationExpressionSymbol) {
            return generateEventConditionEventLogicalExpressionSymbol((EventLogicalOperationExpressionSymbol) expression, componentSymbol, bluePrint);
        } else if (expression instanceof EventPortExpressionValueSymbol) {
            return generateEventConditionEventPortValueSymbol((EventPortExpressionValueSymbol) expression, componentSymbol, bluePrint);
        } else if (expression instanceof EventPortExpressionConnectSymbol) {
            return generateEventConditionEventPortConnectSymbol((EventPortExpressionConnectSymbol) expression, componentSymbol, bluePrint);
        } else if (expression instanceof EventPortExpressionFreeSymbol) {
//            return "( /*"+((EventPortExpressionFreeSymbol) expression).getPortName()+"::free*/ true)";
            return "(" + expression.getName() + "_has_free_request())";
        }

        if (expression instanceof EventBooleanExpressionSymbol) {
            return ((EventBooleanExpressionSymbol) expression).getBooleanValue() ? "true" : "false";
        }

        return "";
    }

    protected static String generateEventConditionEventLogicalExpressionSymbol(EventLogicalOperationExpressionSymbol expression, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint) {
        String result = "( ";
        if (expression.getLeftExpression() != null) {
            result += generateEventCondition(expression.getLeftExpression(), componentSymbol, bluePrint);
        }

        result += expression.getOperator();

        if (expression.getRightExpression() != null) {
            result += generateEventCondition(expression.getRightExpression(), componentSymbol, bluePrint);
        }

        return result + " )";
    }

    protected static String generateEventConditionEventPortValueSymbol(EventPortExpressionValueSymbol expressionValueSymbol, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrintCPP bluePrint) {

        VariablePortValueChecker pvc = new VariablePortValueChecker(expressionValueSymbol.getName());
        bluePrint.addVariable(pvc);

        Optional<EMAPortInstanceSymbol> portSymbol = componentSymbol.getPortInstance(expressionValueSymbol.getName());
        if (portSymbol.isPresent()) {
            String typeNameMontiCar = portSymbol.get().getTypeReference().getName();
            pvc.setVariableType(TypeConverter.getVariableTypeForMontiCarTypeName(typeNameMontiCar, pvc, portSymbol.get()).get());
        }

        addTest(expressionValueSymbol.getPortValue(), pvc);

        return pvc.getNameTargetLanguageFormat() + ".check()";

    }

    protected static String generateEventConditionEventPortConnectSymbol(EventPortExpressionConnectSymbol expressionConnectSymbol, EMAComponentInstanceSymbol componentSymbol, EMAMBluePrint bluePrint) {

        return "(" + expressionConnectSymbol.getName() + "_has_connect_request())";

//        return "(!__"+expressionConnectSymbol.getName()+"_connect_request.empty())";
    }


    //</editor-fold>

    //<editor-fold desc="Generate concrete test for port values">

    public static void addTest(PortValueSymbol pvs, VariablePortValueChecker vpvc) {
        if (pvs instanceof PortValueInputSymbol) {
//            addTestPortValueInputSymbol((PortValueInputSymbol) pvs, vpvc);

            vpvc.addTestSymbol_Equals(((PortValueInputSymbol) pvs).getValueStringRepresentation());

        } else if (pvs instanceof PortValuesArraySymbol) {
            PortValuesArraySymbol ar = (PortValuesArraySymbol) pvs;
            for (int i = 0; i < ar.size(); ++i) {
                addTest(ar.getForIndex(i), vpvc);
            }
        } else if (pvs instanceof PortValuePrecisionSymbol) {

            PortValuePrecisionSymbol ps = (PortValuePrecisionSymbol) pvs;
            String value = ps.getValue().getValueStringRepresentation();
            String prec = ps.getPrecision().getValueStringRepresentation();

            vpvc.addTestSymbol_Range("(" + value + " - " + prec + ")", "(" + value + " + " + prec + ")");
        } else if (pvs instanceof PortValueRangeSymbol) {
            PortValueRangeSymbol ps = (PortValueRangeSymbol) pvs;
            String lower = ps.getLowerBound().getValueStringRepresentation();
            String upper = ps.getUpperBound().getValueStringRepresentation();

            vpvc.addTestSymbol_Range(lower, upper);
        } else if (pvs instanceof PortValueCompareSymbol) {
            PortValueCompareSymbol compare = (PortValueCompareSymbol) pvs;

            vpvc.addTestSymbol_Compare(compare.getOperator(), compare.getCompareValue().getValueStringRepresentation());

        }
    }

    //</editor-fold>
}
