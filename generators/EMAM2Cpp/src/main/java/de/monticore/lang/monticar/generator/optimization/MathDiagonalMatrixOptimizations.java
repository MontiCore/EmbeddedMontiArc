/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.converter.ComponentConverter;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 */
public class MathDiagonalMatrixOptimizations implements MathOptimizationRule {
    MathStatementsSymbol currentMathStatementsSymbol = null;

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol == null) {
            //Do nothing
        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            optimize((MathAssignmentExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixExpression()) {
            optimize((MathMatrixExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            optimize((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.debug("Not handled main Optimize: " + mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(),
                    "optimizeMathExpressionSymbol");
        }
    }

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions, MathStatementsSymbol mathStatementsSymbol) {
        currentMathStatementsSymbol = mathStatementsSymbol;
        optimize(mathExpressionSymbol, precedingExpressions);
    }

    public void optimize(MathAssignmentExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getExpressionSymbol(), precedingExpressions);
    }


    public void optimize(MathArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathExpressionSymbol.getRightExpression(), precedingExpressions);
    }

    public void optimize(MathMatrixExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.isMatrixNameExpression()) {
            optimize((MathMatrixNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isMatrixAccessExpression()) {
            optimize((MathMatrixAccessSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.debug("Not handled: " + mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.
                    getTextualRepresentation(), "optimizeMathMatrixExpr");
        }
    }

    public void optimize(MathMatrixAccessSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.getMathExpressionSymbol().isPresent()) {
            optimize(mathExpressionSymbol.getMathExpressionSymbol().get(), precedingExpressions);
        } else {
            Log.debug("Not handled further", "optimizeMathMatrixAccess");
        }
    }

    public void optimize(MathMatrixNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.getNameToAccess().equals("inv")) {
            //ComponentConverter.currentBluePrint.getMathInformationRegister().isDiagonalMatrix()
            boolean invertsDiagonalMatrix = isDiagonalMatrix(mathExpressionSymbol);
            if (invertsDiagonalMatrix) {
                mathExpressionSymbol.setNameToAccess("invdiag");
            }
            Log.debug("Found inv and replaced with invdiag", "optimizeMathMatrixNameExp");
        } else if (mathExpressionSymbol.getNameToAccess().equals("sqrtm")) {
            boolean isDiagMatrix = isDiagonalMatrix(mathExpressionSymbol);
            if (isDiagMatrix) {
                mathExpressionSymbol.setNameToAccess("sqrtmdiag");
            }
            Log.debug("Found sqrtm and replaced with sqrtdiag", "optimizeMathMatrixNameExp");

        }
        if (mathExpressionSymbol.isMathMatrixAccessOperatorSymbolPresent()) {
            optimize(mathExpressionSymbol.getMathMatrixAccessOperatorSymbol(), precedingExpressions);
        }
    }

    private boolean isDiagonalMatrix(MathMatrixNameExpressionSymbol mathExpressionSymbol) {
        boolean invertsDiagonalMatrix = false;
        if (mathExpressionSymbol.isMathMatrixAccessOperatorSymbolPresent()) {
            //optimize(mathExpressionSymbol.getMathMatrixAccessOperatorSymbol(), precedingExpressions);
            //Log.error(ComponentConverter.currentBluePrint.getMathInformationRegister().getVariable("degree").getProperties().toString());
            String name = getMatrixName((MathMatrixAccessSymbol) mathExpressionSymbol.getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0));//TODO handle all possible cases
            //System.out.println("isDiagonalMatrix: " + name);
            //System.out.println(mathExpressionSymbol.getTextualRepresentation());
            invertsDiagonalMatrix = ComponentConverter.currentBluePrint.getMathInformationRegister().getVariable(name).getProperties().contains("diag");
        }
        return invertsDiagonalMatrix;
    }

    public void optimize(MathMatrixAccessOperatorSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        for (MathExpressionSymbol subExpr : mathExpressionSymbol.getMathMatrixAccessSymbols()) {
            optimize(subExpr, precedingExpressions);
        }
    }

    public static String getMatrixName(MathMatrixAccessSymbol mathExpressionSymbol) {
        assert mathExpressionSymbol.getMathExpressionSymbol().isPresent();
        MathExpressionSymbol curMathExp = mathExpressionSymbol.getMathExpressionSymbol().get();
        if (curMathExp.isValueExpression()) {
            if (((MathValueExpressionSymbol) curMathExp).isNameExpression()) {
                return ((MathNameExpressionSymbol) curMathExp).getNameToResolveValue();
            } else {
                Log.debug("Not handled getMatrixName", "MissingImplementation");
            }
        } else if (curMathExp.isMatrixExpression()) {
            if (((MathMatrixExpressionSymbol) curMathExp).isMatrixNameExpression()) {
                return getMatrixName(((MathMatrixNameExpressionSymbol) curMathExp).getMathMatrixAccessOperatorSymbol().getMathMatrixAccessSymbols().get(0));
            } else {
                Log.debug("Not handled getMatrixName", "MissingImplementation");
            }
        } else {
            Log.debug("Not handled getMatrixName", "MissingImplementation");
        }
        return "";
    }
}
