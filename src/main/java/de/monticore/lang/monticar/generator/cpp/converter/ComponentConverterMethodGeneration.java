package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math.math._symboltable.expression.*;
import de.monticore.lang.math.math._symboltable.matrix.*;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeMathInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.cpp.symbols.MathStringExpression;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ComponentConverterMethodGeneration {
    public static int currentGenerationIndex = 0;
    public static ExpandedComponentInstanceSymbol currentComponentSymbol = null;

    public static void generateExecuteMethod(ExpandedComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Method method = new Method("execute", "void");
        currentComponentSymbol = componentSymbol;
        for (ConnectorSymbol connector : componentSymbol.getConnectors()) {
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
                    ConstantPortSymbol constPort = (ConstantPortSymbol) connector.getSourcePort();
                    Variable v1 = new Variable();
                    v1.setName(constPort.getConstantValue().getValueAsString());
                    Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else if (connector.getTargetPort().isConstant()) {
                    ConstantPortSymbol constPort = (ConstantPortSymbol) connector.getTargetPort();
                    Variable v2 = new Variable();
                    v2.setName(constPort.getConstantValue().getValueAsString());
                    Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);


                    Instruction instruction = new ConnectInstructionCPP(v2, v1);
                    method.addInstruction(instruction);
                } else {
                    Log.error("0xWRONGCONNECTOR the connector is constant but target nor source are constant");
                }
            }
        }
        if (mathStatementsSymbol != null) {
            handleMathStatementGeneration(method, bluePrint, mathStatementsSymbol, generatorCPP, includeStrings);
        }
        bluePrint.addMethod(method);
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
            if (!visitedMathExpressionSymbols.contains(mathExpressionSymbol)) {
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
            }
            handleInstructionReOrdering(method, beginIndex);
        }
        if (generatorCPP.useAlgebraicOptimizations())
            removeUselessVariableDefinitions(method);
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
}
