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
            // add math implementation instructions to method
            List<MathExpressionSymbol> newMathExpressionSymbols = new ArrayList<>();
            MathOptimizer.currentBluePrint = bluePrint;
            int counter = 0;

            for (MathExpressionSymbol mathExpressionSymbol : mathStatementsSymbol.getMathExpressionSymbols()) {
                if (generatorCPP.useAlgebraicOptimizations()) {
                    List<MathExpressionSymbol> precedingExpressions = new ArrayList<>();
                    for (int i = 0; i < counter; ++i)
                        precedingExpressions.add(mathStatementsSymbol.getMathExpressionSymbols().get(i));
                    newMathExpressionSymbols.add(MathOptimizer.applyOptimizations(mathExpressionSymbol, precedingExpressions));
                    ++counter;
                }
                MathFunctionFixer.fixMathFunctions(mathExpressionSymbol, bluePrint);
                String result = ExecuteMethodGenerator.generateExecuteCode(mathExpressionSymbol, includeStrings);
                TargetCodeMathInstruction instruction = new TargetCodeMathInstruction(result, mathExpressionSymbol);
                method.addInstruction(instruction);
            }
        }
        bluePrint.addMethod(method);
    }
}
