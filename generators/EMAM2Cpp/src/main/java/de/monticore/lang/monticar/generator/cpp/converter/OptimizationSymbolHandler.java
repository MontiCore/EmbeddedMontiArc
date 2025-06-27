/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.BaseExecuteMethodGeneratorHandler;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.monticore.lang.monticar.generator.cpp.converter.ExecuteMethodGenerator;
import de.monticore.lang.monticar.generator.cpp.converter.OptimizationSolverConverter;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * Handles code generation for optimization symbols
 *
 */
public class OptimizationSymbolHandler extends BaseExecuteMethodGeneratorHandler {

    // fields
    private String currentOptimizationVariableName = "x";
    private String currentOptimizationVariableMatrixType = "ADMat";

    //Todo:In Progress:
    private List<MathValueSymbol> optimizationVariables = new ArrayList<>();

    @Override
    protected boolean canHandleSymbol(MathExpressionSymbol symbol) {
        boolean canHandle = false;
        if ((symbol instanceof MathOptimizationStatementSymbol) || (symbol instanceof MathMatrixArithmeticValueSymbol))
            canHandle = true;
        return canHandle;
    }

    @Override
    protected String doGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings) {
        String result = "";
        if (symbol instanceof MathOptimizationStatementSymbol) {
            result = OptimizationSolverConverter.getOptimizationExpressionCode((MathOptimizationStatementSymbol) symbol, includeStrings, ComponentConverter.currentBluePrint);
        } else if (symbol instanceof MathMatrixArithmeticValueSymbol) {
            MathMatrixArithmeticValueSymbol matVal = (MathMatrixArithmeticValueSymbol) symbol;
            if (containsOptimizationVariable(matVal))
                result = generateCodeForMatrixDependentOnOptVar(matVal, includeStrings);// do something
            else
                result = ((BaseExecuteMethodGeneratorHandler) getSuccessor()).doGenerateExecuteCode(symbol, includeStrings);
        } else {
            Log.error(String.format("%s: Case not handled: %s", getRole(), symbol.getTextualRepresentation()), symbol.getSourcePosition());
        }
        return result;
    }

    private String generateCodeForMatrixDependentOnOptVar(MathMatrixArithmeticValueSymbol matVal, List<String> includeStrings) {
        String result;
        if ((matVal.getVectors().size() == 1) || (matVal.getVectors().get(0).getMathMatrixAccessSymbols().size() == 1))
            result = generateCodeForVecDependentOnOptVar(matVal, includeStrings);
        else {
            StringBuilder matrixInitSB = new StringBuilder();
            for (MathMatrixAccessOperatorSymbol vec : matVal.getVectors()) {
                matrixInitSB.append("{");
                for (MathMatrixAccessSymbol elem : vec.getMathMatrixAccessSymbols()) {
                    matrixInitSB.append(ExecuteMethodGenerator.generateExecuteCode(elem, includeStrings));
                    matrixInitSB.append(",");
                }
                matrixInitSB.deleteCharAt(matrixInitSB.length() - 1);
                matrixInitSB.append("},");
            }
            matrixInitSB.deleteCharAt(matrixInitSB.length() - 1);
            result = String.format("%s({%s})", getCurrentOptimizationVariableMatrixType(), matrixInitSB.toString());
        }
        return result;
    }

    private String generateCodeForVecDependentOnOptVar(MathMatrixArithmeticValueSymbol matVal, List<String> includeStrings) {
        StringBuilder matrixInitSB = new StringBuilder();
        for (MathMatrixAccessOperatorSymbol vec : matVal.getVectors()) {
            for (MathMatrixAccessSymbol elem : vec.getMathMatrixAccessSymbols()) {
                matrixInitSB.append(ExecuteMethodGenerator.generateExecuteCode(elem, includeStrings));
                matrixInitSB.append(",");
            }
        }
        matrixInitSB.deleteCharAt(matrixInitSB.length() - 1);
        String result = String.format("%s({%s})", getCurrentOptimizationVariableMatrixType(), matrixInitSB.toString());
        if ((matVal.getVectors().size() > 1) && (matVal.getVectors().get(0).getMathMatrixAccessSymbols().size() == 1))
            result = String.format("(%s.t())", result); // transpose to colvec
        return result;
    }

    private boolean containsOptimizationVariable(MathMatrixArithmeticValueSymbol matVal) {
        boolean result = false;
        for (MathMatrixAccessOperatorSymbol vec : matVal.getVectors()) {
            for (MathMatrixAccessSymbol elem : vec.getMathMatrixAccessSymbols()) {
                String textRep = elem.getTextualRepresentation();
                for (String varName : getCurrentOptimizationVariableNames())
                    if (textRep.contentEquals(varName) || textRep.contains(varName + "("))
                        return true;
            }
        }
        return result;
    }

    public List<String> getCurrentOptimizationVariableNames() {
        List<String> varNames = new ArrayList<>();
        for (MathValueSymbol var : optimizationVariables){
            varNames.add(var.getName());
        }
        return varNames;
    }

    public void setOptimizationVariables(List<MathValueSymbol> optimizationVariables) {
        this.optimizationVariables = optimizationVariables;
    }

    //public void setCurrentOptimizationVariableName(String currentOptimizationVariableName) {
    //    this.currentOptimizationVariableName = currentOptimizationVariableName;
    //}

    public String getCurrentOptimizationVariableMatrixType() {
        return currentOptimizationVariableMatrixType;
    }

    public void setCurrentOptimizationVariableMatrixType(String currentOptimizationVariableMatrixType) {
        this.currentOptimizationVariableMatrixType = currentOptimizationVariableMatrixType;
    }

    @Override
    public String getRole() {
        return "OptimizationSymbolHandler";
    }
}
