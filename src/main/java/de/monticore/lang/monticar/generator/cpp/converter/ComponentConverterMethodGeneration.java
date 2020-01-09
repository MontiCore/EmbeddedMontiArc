/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.*;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.monticore.lang.monticar.ts.MCTypeSymbol;
import de.monticore.lang.monticar.ts.references.MCTypeReference;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

/**
 */
public class ComponentConverterMethodGeneration {
    public static int currentGenerationIndex = 0;
    public static EMAComponentInstanceSymbol currentComponentSymbol = null;

    public static Method generateExecuteMethod(EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {


        currentComponentSymbol = componentSymbol;
/*
        if(componentSymbol instanceof EMADynamicComponentInstanceSymbol){
            EMADynamicComponentInstanceSymbol dynComp = (EMADynamicComponentInstanceSymbol)componentSymbol;
            if(dynComp.getEventHandlers().size() > 0) {
                Method execute = new Method("execute", "void");
                Method inner = new Method("execute_inner", "void");
                inner.setPublic(false);
                generateExecuteMethodInner(inner, componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);

                EventHandlerMethodsGenerator.generateInputsOfEventHandlers(componentSymbol, bluePrint,execute);

                if (!inner.getInstructions().isEmpty()){
                    execute.addInstruction(new TargetCodeInstruction("//--------------------\n"));
                    execute.addInstruction(new TargetCodeInstruction("execute_inner();\n"));
                    execute.addInstruction(new TargetCodeInstruction("//--------------------\n"));
                    bluePrint.addMethod(inner);
                }

                EventHandlerMethodsGenerator.generateOutputsOfEventHandlers(componentSymbol, bluePrint,execute);

                bluePrint.addMethod(execute);
                return;
            }
        }
*/
        Method exMethod = new Method("execute", "void");
        generateExecuteMethodInner(exMethod,componentSymbol, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
//        bluePrint.addMethod(exMethod);
        return exMethod;
    }

    protected static void generateExecuteMethodInner(Method method, EMAComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings){


        Collection<EMAConnectorInstanceSymbol> connectors = componentSymbol.getConnectorInstances();
        generateConnectors(connectors, bluePrint, method);



        if (mathStatementsSymbol != null) {
            handleMathStatementGeneration(method, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
        }

    }

    public static void generateConnectors(Collection<EMAConnectorInstanceSymbol> connectors, BluePrintCPP bluePrint, Method method){
        for (EMAConnectorInstanceSymbol connector : connectors) {
            if (!connector.isConstant()) {
                Log.info("source:" + connector.getSource() + " target:" + connector.getTarget(), "Port info:");
                Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);
                Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);
                Log.info("v1: " + v1.getName() + " v2: " + v2.getName(), "Variable Info:");
                Log.info("v1: " + v1.getNameTargetLanguageFormat() + " v2: " + v2.getNameTargetLanguageFormat(), "Variable Info:");

                Instruction instruction = new ConnectInstructionCPP(v2, v1);
                method.addInstruction(instruction);
            } else {
                if (connector.getSourcePort().isConstant()) {
                    EMAPortInstanceSymbol constPort =  connector.getSourcePort();
                    Variable v1 = new Variable();
                    v1.setName(constPort.getConstantValue().get().getValueAsString());
                    Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else if (connector.getTargetPort().isConstant()) {
                    EMAPortInstanceSymbol constPort = connector.getTargetPort();
                    Variable v2 = new Variable();
                    v2.setName(constPort.getConstantValue().get().getValueAsString());
                    Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else {
                    Log.error("0xWRONGCONNECTOR the connector is constant but target nor source are constant");
                }
            }
        }
    }

    private static List<MathExpressionSymbol> visitedMathExpressionSymbols = new ArrayList<>();
    private static boolean swapNextInstructions = false;

    private static void handleMathStatementGeneration(Method method, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        // add math implementation instructions to method
        List<MathExpressionSymbol> newMathExpressionSymbols = new ArrayList<>();
        MathOptimizer.currentBluePrint = bluePrint;
        int counter = 0;
        visitedMathExpressionSymbols.clear();
        //int lastIndex = 0;
        for (currentGenerationIndex = 0; currentGenerationIndex < mathStatementsSymbol.getMathExpressionSymbols().size(); ++currentGenerationIndex) {
            int beginIndex = currentGenerationIndex;
            MathExpressionSymbol mathExpressionSymbol = mathStatementsSymbol.getMathExpressionSymbols().get(currentGenerationIndex);
            if (!containsExactObject(visitedMathExpressionSymbols, mathExpressionSymbol)) {
                if (generatorCPP.useAlgebraicOptimizations()) {
                    List<MathExpressionSymbol> precedingExpressions = new ArrayList<>();
                    for (int i = 0; i < counter; ++i)
                        precedingExpressions.add(mathStatementsSymbol.getMathExpressionSymbols().get(i));
                    if (mathExpressionSymbol != visitedMathExpressionSymbols)
                        newMathExpressionSymbols.add(MathOptimizer.applyOptimizations(mathExpressionSymbol, precedingExpressions, mathStatementsSymbol));
                    ++counter;
                }
                generateInstruction(method, mathExpressionSymbol, bluePrint, includeStrings);
                //lastIndex = currentGenerationIndex;
                //fix argumentReturnFunctionsInstructions

            }
            handleInstructionReOrdering(method, beginIndex);
        }
        if (generatorCPP.useAlgebraicOptimizations())
            removeUselessVariableDefinitions(method);
    }

    private static boolean containsExactObject(Collection collection, Object obj){
        for(Object other : collection){
            //equals is not used on purpose!
            //the jvm object id must be the same!
            if(other == obj) return true;
        }

        return false;
    }

    private static void removeUselessVariableDefinitions(Method method) {
        List<Instruction> instructionsToRemove = new ArrayList<>();
        for (Instruction instruction1 : method.getInstructions()) {
            if (instruction1 instanceof TargetCodeMathInstruction) {
                TargetCodeMathInstruction targetIns = (TargetCodeMathInstruction) instruction1;
                if (targetIns.getAddedVariable().isPresent()) {
                    boolean usesVariable = false;
                    for (Instruction instruction2 : method.getInstructions()) {
                        if (instruction2 instanceof TargetCodeMathInstruction) {
                            //Log.info("test", "removeUselessVariable:");
                            if (((TargetCodeMathInstruction) instruction2).getUsedVariables().contains(
                                    targetIns.getAddedVariable().get())) {
                                usesVariable = true;
                                break;
                            }
                        }
                    }
                    if (!usesVariable) {
                        instructionsToRemove.add(instruction1);
                    }
                }
            }
        }

        for (Instruction instructionToRemove : instructionsToRemove) {
            method.getInstructions().remove(instructionToRemove);
            Log.info(instructionToRemove.getTargetLanguageInstruction(), "Removed Instruction:");
        }
        if (instructionsToRemove.size() > 0) {
            removeUselessVariableDefinitions(method);
        }
    }

    private static void generateInstruction(Method method, MathExpressionSymbol
            mathExpressionSymbol, BluePrintCPP bluePrint, List<String> includeStrings/*, int lastIndex*/) {
        MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrint);
        String result = ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
        for (MathCommand mathCommand : ComponentConverter.usedMathCommand)
        if(mathCommand != null) {
            String argumentReturnFunctionName = mathCommand.getMathCommandName();
            if (mathCommand.isArgumentReturnMathCommand() && result.contains(argumentReturnFunctionName)) {
                result = fixArgumentReturnInstruction(result, method, mathExpressionSymbol, bluePrint);
            }
        }
        TargetCodeMathInstruction instruction = new TargetCodeMathInstruction(result, mathExpressionSymbol);
        Log.info(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "GenerateSymbol:");
        if (mathExpressionSymbol instanceof MathValueSymbol) {
            instruction.setAddedVariable(mathExpressionSymbol.getName());
        }
        calculateUsedVariables(mathExpressionSymbol, instruction);
        Log.info(instruction.getTargetCode(), "Instruction:");
        for (String name : instruction.getUsedVariables()) {
            Log.info(name, "Used Variables:");
        }
        method.addInstruction(instruction);
        visitedMathExpressionSymbols.add(mathExpressionSymbol);
        //Log.debug("lastIndex: " + lastIndex + " current: " + currentGenerationIndex, "ComponentConverterMethodGeneration");
    }

    private static void calculateUsedVariables(MathExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        if (mathExpressionSymbol != null) {
            Log.info(mathExpressionSymbol.getTextualRepresentation(), "EXP:");
        }
        if (mathExpressionSymbol == null) {
        } else if (mathExpressionSymbol instanceof MathValueSymbol) {
            calculateUsedVariables((MathValueSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            calculateUsedVariables((MathAssignmentExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            calculateUsedVariables((MathArithmeticExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            calculateUsedVariables((MathMatrixExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isValueExpression()) {
            if (((MathValueExpressionSymbol) mathExpressionSymbol).isNameExpression()) {
                calculateUsedVariables((MathNameExpressionSymbol) mathExpressionSymbol, instruction);
            }
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            calculateUsedVariables(mathExpressionSymbol.getRealMathExpressionSymbol(), instruction);
        } else if (mathExpressionSymbol.isForLoopExpression()) {
            calculateUsedVariables((MathForLoopExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol instanceof MathStringExpression) {
            for (MathExpressionSymbol expressionSymbol : ((MathStringExpression) mathExpressionSymbol).getPreviousExpressionSymbols())
                calculateUsedVariables(expressionSymbol, instruction);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName() + " " +
                    mathExpressionSymbol.getTextualRepresentation(), "Not handled calculateUsedVariables1:");
        }
    }

    private static void calculateUsedVariables(MathForLoopExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getForLoopHead().getMathExpression(), instruction);
        for (MathExpressionSymbol mathExpressionSymbol1 : mathExpressionSymbol.getForLoopBody()) {
            calculateUsedVariables(mathExpressionSymbol1, instruction);
        }
    }

    private static void calculateUsedVariables(MathMatrixExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            calculateUsedVariables((MathMatrixNameExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixArithmeticExpression()) {
            calculateUsedVariables((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixAccessExpression()) {
            calculateUsedVariables((MathMatrixAccessSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol.isMatrixVectorExpression()) {
            calculateUsedVariables((MathMatrixVectorExpressionSymbol) mathExpressionSymbol, instruction);
        } else if (mathExpressionSymbol instanceof MathMatrixAccessOperatorSymbol) {
            calculateUsedVariables((MathMatrixAccessOperatorSymbol) mathExpressionSymbol, instruction);
            //calculateUsedVariables((MathMatrixPreOperatorExpressionSymbol) mathExpressionSymbol, instruction);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName() + " " +
                    mathExpressionSymbol.getTextualRepresentation(), "Not handled calculateUsedVariables2:");
        }
    }

    private static void calculateUsedVariables(MathMatrixNameExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        instruction.addUsedVariable(mathExpressionSymbol.getNameToAccess());
        calculateUsedVariables(mathExpressionSymbol.getMathMatrixAccessOperatorSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathMatrixAccessOperatorSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        //parent calculateUsedVariables(mathExpressionSymbol.getMathMatrixNameExpressionSymbol(), instruction);
        for (MathExpressionSymbol expSymbol : mathExpressionSymbol.getMathMatrixAccessSymbols())
            calculateUsedVariables(expSymbol, instruction);
    }


    private static void calculateUsedVariables(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getLeftExpression(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getRightExpression(), instruction);
    }


    private static void calculateUsedVariables(MathMatrixAccessSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.getMathExpressionSymbol().isPresent()) {
            calculateUsedVariables(mathExpressionSymbol.getMathExpressionSymbol().get(), instruction);
        }
    }


    private static void calculateUsedVariables(MathMatrixVectorExpressionSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        calculateUsedVariables(mathExpressionSymbol.getStart(), instruction);
        if (mathExpressionSymbol.getStep().isPresent())
            calculateUsedVariables(mathExpressionSymbol.getStep().get(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getEnd(), instruction);
    }

    private static void calculateUsedVariables(MathValueSymbol mathExpressionSymbol, TargetCodeMathInstruction
            instruction) {
        if (mathExpressionSymbol.getValue() != null)
            calculateUsedVariables(mathExpressionSymbol.getValue().getRealMathExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathArithmeticExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        calculateUsedVariables(mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), instruction);
        calculateUsedVariables(mathExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathAssignmentExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        calculateUsedVariables(mathExpressionSymbol.getExpressionSymbol(), instruction);
    }

    private static void calculateUsedVariables(MathNameExpressionSymbol
                                                       mathExpressionSymbol, TargetCodeMathInstruction instruction) {
        instruction.addUsedVariable(mathExpressionSymbol.getNameToResolveValue());
    }

    private static void handleInstructionReOrdering(Method method, int beginIndex) {
        if (swapNextInstructions) {
            swapNextInstructions = false;
            //Log.error("ad");
            Instruction lastInstruction = method.getInstructions().get(currentGenerationIndex);
            method.getInstructions().remove(currentGenerationIndex);
            method.addInstruction(lastInstruction);
        }
        if (beginIndex != currentGenerationIndex) swapNextInstructions = true;
    }

    private static String fixArgumentReturnInstruction(String instruction, Method method, MathExpressionSymbol mathExpressionSymbol, BluePrintCPP bluePrintCPP){
    String newInstruction = "";
    if(instruction.contains("=")){
        int indexOfEqualOperator = instruction.indexOf("=");
        String beforeEqualOperatorSubString = instruction.substring(0,indexOfEqualOperator-1);
        String afterEqualOperatorSubString = instruction.substring(indexOfEqualOperator+2);

        if(afterEqualOperatorSubString.contains(",")){
            int indexOfCommaOperator  = afterEqualOperatorSubString.indexOf(",");
            newInstruction = afterEqualOperatorSubString.substring(0,indexOfCommaOperator) + ", " + beforeEqualOperatorSubString +
                    afterEqualOperatorSubString.substring(indexOfCommaOperator);
        }else{
            int indexOfBracket = afterEqualOperatorSubString.indexOf(")");
            newInstruction = afterEqualOperatorSubString.substring(0,indexOfBracket) + ", " + beforeEqualOperatorSubString +
                    afterEqualOperatorSubString.substring(indexOfBracket);
        }
        newInstruction = removeBracket(newInstruction);
        return newInstruction;
    }
    return instruction;
    }

    private static String removeBracket(String instruction){
        String newInstruction = "";
        if(instruction.indexOf("(") == 0){
            int indexOfLastBracket = instruction.lastIndexOf(")");
            newInstruction = instruction.substring(1,indexOfLastBracket) + instruction.substring(indexOfLastBracket+1);
            return newInstruction;
        }
        return instruction;
    }
}
