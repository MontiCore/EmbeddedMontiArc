package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.Instruction;
import de.monticore.lang.monticar.generator.Method;
import de.monticore.lang.monticar.generator.TargetCodeMathInstruction;
import de.monticore.lang.monticar.generator.Variable;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.MathFunctionFixer;
import de.monticore.lang.monticar.generator.cpp.instruction.ConnectInstructionCPP;
import de.monticore.lang.monticar.generator.optimization.MathOptimizer;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class ComponentConverterMethodGeneration {
    public static int currentGenerationIndex = 0;

    public static void generateExecuteMethod(ExpandedComponentInstanceSymbol componentSymbol, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        Method method = new Method("execute", "void");

        for (ConnectorSymbol connector : componentSymbol.getConnectors()) {
            if (!connector.isConstant()) {
                Log.info("source:" + connector.getSource() + " target:" + connector.getTarget(), "Port info:");
                Variable v1 = PortConverter.getVariableForPortSymbol(connector, connector.getSource(), bluePrint);
                Variable v2 = PortConverter.getVariableForPortSymbol(connector, connector.getTarget(), bluePrint);


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

    private static void handleMathStatementGeneration(Method method, BluePrintCPP bluePrint, MathStatementsSymbol mathStatementsSymbol, GeneratorCPP generatorCPP, List<String> includeStrings) {
        // add math implementation instructions to method
        List<MathExpressionSymbol> newMathExpressionSymbols = new ArrayList<>();
        MathOptimizer.currentBluePrint = bluePrint;
        int counter = 0;

        List<MathExpressionSymbol> visitedMathExpressionSymbol = new ArrayList<>();
        int lastIndex = 0;
        boolean swapNextInstructions = false;
        for (currentGenerationIndex = 0; currentGenerationIndex < mathStatementsSymbol.getMathExpressionSymbols().size(); ++currentGenerationIndex) {
            int beginIndex = currentGenerationIndex;
            MathExpressionSymbol mathExpressionSymbol = mathStatementsSymbol.getMathExpressionSymbols().get(currentGenerationIndex);
            if (!visitedMathExpressionSymbol.contains(mathExpressionSymbol)) {
                if (generatorCPP.useAlgebraicOptimizations()) {
                    List<MathExpressionSymbol> precedingExpressions = new ArrayList<>();
                    for (int i = 0; i < counter; ++i)
                        precedingExpressions.add(mathStatementsSymbol.getMathExpressionSymbols().get(i));
                    if (mathExpressionSymbol != visitedMathExpressionSymbol)
                        newMathExpressionSymbols.add(MathOptimizer.applyOptimizations(mathExpressionSymbol, precedingExpressions, mathStatementsSymbol));
                    ++counter;
                }
                MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrint);
                String result = ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
                TargetCodeMathInstruction instruction = new TargetCodeMathInstruction(result, mathExpressionSymbol);
                method.addInstruction(instruction);
                visitedMathExpressionSymbol.add(mathExpressionSymbol);
                System.out.println("lastIndex: " + lastIndex + " current: " + currentGenerationIndex);
                lastIndex = currentGenerationIndex;
            }
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
}
