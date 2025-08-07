/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Rule that transforms ABC to A(BC) if this reduces the amount of operations required
 *
 */
public class MathMatrixMultiplicationOrder implements MathOptimizationRule {

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol == null) {

        } else if (mathExpressionSymbol.isMatrixExpression()) {
            optimize((MathMatrixExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isAssignmentExpression()) {
            optimize((MathAssignmentExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isParenthesisExpression()) {
            optimize((MathParenthesisExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isArithmeticExpression()) {
            optimize((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isValueExpression()) {
            MathValueExpressionSymbol mathValueExpressionSymbol = (MathValueExpressionSymbol) mathExpressionSymbol;
            if (mathValueExpressionSymbol.isNameExpression())
                optimize((MathNameExpressionSymbol) mathExpressionSymbol, precedingExpressions);
            else if (mathExpressionSymbol.isValueExpression()) {
                //optimize((MathValueSymbol) mathExpressionSymbol, precedingExpressions);
            } else if (((MathValueExpressionSymbol) mathExpressionSymbol).isNumberExpression()) {
                //optimize((MathNumberExpressionSymbol)mathExpressionSymbol,precedingExpressions);
            }
        } else if (mathExpressionSymbol.isForLoopExpression()) {
            optimize((MathForLoopExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName(), "Symbol name:");
            Log.debug("Optimizer case not handled", "Case not handled!");
        }
    }

    public void optimize(MathForLoopExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        //TODO enable head optimization later
        for (MathExpressionSymbol subExp : mathExpressionSymbol.getForLoopBody()) {
            optimize(subExp, precedingExpressions);
        }
    }

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions, MathStatementsSymbol mathStatementsSymbol) {
        optimize(mathExpressionSymbol, precedingExpressions);
    }

    public void optimize(MathNameExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        // do nothing
    }

    public void optimizeSubExpressions(IArithmeticExpression mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathExpressionSymbol.getRightExpression(), precedingExpressions);
    }

    public void optimize(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimizeSubExpressions(mathArithmeticExpressionSymbol, precedingExpressions);
        if (mathArithmeticExpressionSymbol.getMathOperator().equals("*")) {
            while (optimizeMatrixMultiplication(mathArithmeticExpressionSymbol, precedingExpressions)) {
                //do nothing
            }

        }
    }

    public void optimize(MathParenthesisExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getMathExpressionSymbol(), precedingExpressions);
    }

    public void optimize(MathAssignmentExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getExpressionSymbol(), precedingExpressions);
    }

    public void optimize(MathMatrixExpressionSymbol matrixExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (matrixExpressionSymbol.isMatrixArithmeticExpression()) {
            optimize((MathMatrixArithmeticExpressionSymbol) matrixExpressionSymbol, precedingExpressions);
        } else if (matrixExpressionSymbol.isMatrixNameExpression()) {
            optimize((MathMatrixNameExpressionSymbol) matrixExpressionSymbol, precedingExpressions);
        } else {
            Log.info(matrixExpressionSymbol.getClass().getName(), "Symbol name:");
            Log.debug("Optimizer case not handled", "Case not handled!");

        }
    }

    public void optimize(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathExpressionSymbol.getRightExpression(), precedingExpressions);
    }

    public void optimize(MathMatrixNameExpressionSymbol matrixNameExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        //nothing to do in this rule
    }

   /* public void optimize(MathMatrixArithmeticExpressionSymbol matrixArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(matrixArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(matrixArithmeticExpressionSymbol.getRightExpression(), precedingExpressions);
        if (MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            MathExpressionSymbol firstExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol secondExpressionSymbol = MathOptimizer.getRightExpressionSymbol(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol thirdExpressionSymbol = matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol();
            Log.info(firstExpressionSymbol.getTextualRepresentation(), "first:");
            Log.info(secondExpressionSymbol.getTextualRepresentation(), "second:");
            Log.info(thirdExpressionSymbol.getTextualRepresentation(), "third:");

            int normalOperationsAmount = MathOptimizer.getEstimatedOperations(matrixArithmeticExpressionSymbol, precedingExpressions);
            MathArithmeticExpressionSymbol newExpression = new MathArithmeticExpressionSymbol();
            newExpression.setMathOperator("*");
            newExpression.setLeftExpression(new MathParenthesisExpressionSymbol(firstExpressionSymbol));
            MathArithmeticExpressionSymbol secondThirdExpression = new MathArithmeticExpressionSymbol();
            secondThirdExpression.setMathOperator("*");
            secondThirdExpression.setLeftExpression(secondExpressionSymbol);
            secondThirdExpression.setRightExpression(thirdExpressionSymbol);
            newExpression.setRightExpression(new MathParenthesisExpressionSymbol(secondThirdExpression));

            int operationsRewrite = MathOptimizer.getEstimatedOperations(newExpression, precedingExpressions);
            Log.info(newExpression.getTextualRepresentation(), "Rewrite Equation:");
            Log.info(operationsRewrite + "", "operations rewrite:");
            Log.info(matrixArithmeticExpressionSymbol.getTextualRepresentation(), "Standard Equation:");
            Log.info(normalOperationsAmount + "", "operations standard:");
            if (normalOperationsAmount > operationsRewrite) {
                matrixArithmeticExpressionSymbol.setLeftExpression(newExpression.getLeftExpression());
                matrixArithmeticExpressionSymbol.setRightExpression(newExpression.getRightExpression());
            }
        } else if (MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            MathExpressionSymbol firstExpressionSymbol = matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol();
            MathExpressionSymbol secondExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol thirdExpressionSymbol = MathOptimizer.getRightExpressionSymbol(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);


            int normalOperationsAmount = MathOptimizer.getEstimatedOperations(matrixArithmeticExpressionSymbol, precedingExpressions);
            MathArithmeticExpressionSymbol newExpression = new MathArithmeticExpressionSymbol();
            newExpression.setMathOperator("*");
            newExpression.setLeftExpression(new MathParenthesisExpressionSymbol(firstExpressionSymbol));
            MathArithmeticExpressionSymbol secondThirdExpression = new MathArithmeticExpressionSymbol();
            secondThirdExpression.setMathOperator("*");
            secondThirdExpression.setLeftExpression(secondExpressionSymbol);
            secondThirdExpression.setRightExpression(thirdExpressionSymbol);
            newExpression.setRightExpression(new MathParenthesisExpressionSymbol(secondThirdExpression));

            int operationsRewrite = MathOptimizer.getEstimatedOperations(newExpression, precedingExpressions);
            Log.info(newExpression.getTextualRepresentation(), "Rewrite Equation:");
            Log.info(operationsRewrite + "", "operations rewrite:");
            Log.info(matrixArithmeticExpressionSymbol.getTextualRepresentation(), "Standard Equation:");
            Log.info(normalOperationsAmount + "", "operations standard:");
            if (normalOperationsAmount > operationsRewrite) {
                matrixArithmeticExpressionSymbol.setLeftExpression(newExpression.getLeftExpression());
                matrixArithmeticExpressionSymbol.setRightExpression(newExpression.getRightExpression());
            }
        }
    }*/
/*
    public void optimizeMatrixMultiplication(MathMatrixArithmeticExpressionSymbol matrixArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions) && MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            Log.error("b");
        }
    }*/
/*
    public void optimizeMatrixMultiplication(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathArithmeticExpressionSymbol.getRightExpression(), precedingExpressions);
        if (MathOptimizer.isArithmeticExpression(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            Log.info(mathArithmeticExpressionSymbol.getTextualRepresentation(), "Whole Expression:");
            MathExpressionSymbol firstExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(MathOptimizer.getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions), precedingExpressions);
            MathExpressionSymbol secondExpressionSymbol = MathOptimizer.getRightExpressionSymbol(MathOptimizer.getCurrentAssignment(mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions), precedingExpressions);
            MathExpressionSymbol thirdExpressionSymbol = mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol();
            Log.info(firstExpressionSymbol.getTextualRepresentation(), "first:");
            Log.info(secondExpressionSymbol.getTextualRepresentation(), "second:");
            Log.info(thirdExpressionSymbol.getTextualRepresentation(), "third:");

            int normalOperationsAmount = MathOptimizer.getEstimatedOperations((MathArithmeticExpressionSymbol) mathArithmeticExpressionSymbol, precedingExpressions);

            MathArithmeticExpressionSymbol newExpression = new MathArithmeticExpressionSymbol();
            newExpression.setMathOperator("*");
            newExpression.setLeftExpression(new MathParenthesisExpressionSymbol(firstExpressionSymbol));
            MathArithmeticExpressionSymbol secondThirdExpression = new MathArithmeticExpressionSymbol();
            secondThirdExpression.setMathOperator("*");
            secondThirdExpression.setLeftExpression(secondExpressionSymbol);
            secondThirdExpression.setRightExpression(thirdExpressionSymbol);
            newExpression.setRightExpression(new MathParenthesisExpressionSymbol(secondThirdExpression));

            int operationsRewrite = MathOptimizer.getEstimatedOperations(newExpression, precedingExpressions);
            Log.info(newExpression.getTextualRepresentation(), "Rewrite Equation:");
            Log.info(operationsRewrite + "", "operations rewrite:");
            Log.info(mathArithmeticExpressionSymbol.getTextualRepresentation(), "Standard Equation:");
            Log.info(normalOperationsAmount + "", "operations standard:");
            if (normalOperationsAmount > operationsRewrite) {
                Log.info("True", "Rewrite:");
                mathArithmeticExpressionSymbol.setLeftExpression(newExpression.getLeftExpression());
                mathArithmeticExpressionSymbol.setRightExpression(newExpression.getRightExpression());
            }
        } else if (MathOptimizer.isArithmeticExpression(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            MathExpressionSymbol firstExpressionSymbol = mathArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol();
            MathExpressionSymbol secondExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(MathOptimizer.getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions), precedingExpressions);
            MathExpressionSymbol thirdExpressionSymbol = MathOptimizer.getRightExpressionSymbol(MathOptimizer.getCurrentAssignment(mathArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions), precedingExpressions);
            Log.info(firstExpressionSymbol.getTextualRepresentation(), "first:");
            Log.info(secondExpressionSymbol.getTextualRepresentation(), "second:");
            Log.info(thirdExpressionSymbol.getTextualRepresentation(), "third:");

            int normalOperationsAmount = MathOptimizer.getEstimatedOperations(mathArithmeticExpressionSymbol, precedingExpressions);
            MathArithmeticExpressionSymbol newExpression = new MathArithmeticExpressionSymbol();
            newExpression.setMathOperator("*");
            newExpression.setLeftExpression(new MathParenthesisExpressionSymbol(firstExpressionSymbol));
            MathArithmeticExpressionSymbol secondThirdExpression = new MathArithmeticExpressionSymbol();
            secondThirdExpression.setMathOperator("*");
            secondThirdExpression.setLeftExpression(secondExpressionSymbol);
            secondThirdExpression.setRightExpression(thirdExpressionSymbol);
            newExpression.setRightExpression(new MathParenthesisExpressionSymbol(secondThirdExpression));

            int operationsRewrite = MathOptimizer.getEstimatedOperations(newExpression, precedingExpressions);
            Log.info(newExpression.getTextualRepresentation(), "Rewrite Equation:");
            Log.info(operationsRewrite + "", "operations rewrite:");
            Log.info(mathArithmeticExpressionSymbol.getTextualRepresentation(), "Standard Equation:");
            Log.info(normalOperationsAmount + "", "operations standard:");
            if (normalOperationsAmount > operationsRewrite) {
                Log.info("True", "Rewrite:");
                mathArithmeticExpressionSymbol.setLeftExpression(newExpression.getLeftExpression());
                mathArithmeticExpressionSymbol.setRightExpression(newExpression.getRightExpression());
            }
        }
    }*/

    public boolean optimizeMatrixMultiplication(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {

        MathExpressionSymbol oldMathArithmeticExpressionSymbol = getInflatedExpression(mathArithmeticExpressionSymbol, precedingExpressions);
        List<MathExpressionSymbol> newExps = getNewExpressions(mathArithmeticExpressionSymbol, precedingExpressions);


        MathExpressionSymbol newExpression = getMostEfficientExpression(newExps);

        Log.info(oldMathArithmeticExpressionSymbol.getTextualRepresentation(), "oldExpression inflated");
        String debugHelpStringNew = newExpression.getTextualRepresentation();
        String debugHelpStringOld = oldMathArithmeticExpressionSymbol.getTextualRepresentation();
        Log.info(newExpression.getTextualRepresentation(), "newExpression:");
        Log.info(newExpression.getTextualRepresentation(), "Term Rewritten:");
        long operationsStandard = MathOptimizer.getEstimatedOperations(oldMathArithmeticExpressionSymbol, precedingExpressions);
        long operationsRewrite = MathOptimizer.getEstimatedOperations(newExpression, precedingExpressions);

        Log.info(operationsStandard + "", "Operations standard term:");
        Log.info(operationsRewrite + "", "Operations rewrite term:");
        if (operationsStandard > operationsRewrite) {
            //Log.info("True", "Rewrite:");
            if (MathOptimizer.isArithmeticExpression(newExpression.getRealMathExpressionSymbol(), "*", precedingExpressions)) {
                mathArithmeticExpressionSymbol.setLeftExpression(((MathArithmeticExpressionSymbol) newExpression.getRealMathExpressionSymbol()).getLeftExpression());
                mathArithmeticExpressionSymbol.setRightExpression(((MathArithmeticExpressionSymbol) newExpression.getRealMathExpressionSymbol()).getRightExpression());
                return true;
            }
        }
        return false;
    }

    public MathArithmeticExpressionSymbol getInflatedExpression(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        MathArithmeticExpressionSymbol newExp = new MathArithmeticExpressionSymbol();
        newExp.setMathOperator(mathArithmeticExpressionSymbol.getMathOperator());
        MathExpressionSymbol inflatedLeftExpression = getInflatedExpression(mathArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions);
        MathExpressionSymbol inflatedRightExpression = getInflatedExpression(mathArithmeticExpressionSymbol.getRightExpression(), precedingExpressions);
        newExp.setLeftExpression(inflatedLeftExpression);
        newExp.setRightExpression(inflatedRightExpression);

        if (!newExp.getTextualRepresentation().equals(mathArithmeticExpressionSymbol.getTextualRepresentation())) {
            MathArithmeticExpressionSymbol newerExp = getInflatedExpression(newExp, precedingExpressions);

            while (!newerExp.getTextualRepresentation().equals(newExp.getTextualRepresentation())) {
                MathArithmeticExpressionSymbol tmp = newerExp;
                newerExp = getInflatedExpression(newExp, precedingExpressions);
                newExp = tmp;
            }
            return newExp;
        }
        return mathArithmeticExpressionSymbol;
    }

    public MathExpressionSymbol getInflatedExpression(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.isArithmeticExpression()) {
            return getInflatedExpression((MathArithmeticExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else if (mathExpressionSymbol.isValueExpression()) {
            return getInflatedExpression((MathValueExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        } else {
            Log.info(mathExpressionSymbol.getClass().getName() + " " + mathExpressionSymbol.getTextualRepresentation(), "Inflation Not handled");
        }
        return mathExpressionSymbol;
    }

    public MathExpressionSymbol getInflatedExpression(MathValueExpressionSymbol mathExp, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExp.isNameExpression()) {
            MathNameExpressionSymbol mathNameExpressionSymbol = (MathNameExpressionSymbol) mathExp;
            for (MathExpressionSymbol mathSymbol : precedingExpressions) {
                Log.info(mathSymbol.getClass().getName() + " " + mathSymbol.getTextualRepresentation(), "precedingExp");
                if (mathSymbol.isValueExpression()) {
                    MathValueSymbol valueSymbol = (MathValueSymbol) mathSymbol;
                    Log.info(valueSymbol.getName() + " " + mathNameExpressionSymbol.getNameToResolveValue(), "name check");

                    if (valueSymbol.getName().equals(mathNameExpressionSymbol.getNameToResolveValue())) {
                        return valueSymbol.getValue();
                    }
                }
            }
        }

        Log.info(mathExp.getClass().
                getName() + " " + mathExp.getTextualRepresentation(), "Inflation not handled:");
        return mathExp;
    }

    public List<MathExpressionSymbol> getNewExpressions(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        List<MathExpressionSymbol> mathExpressionSymbols = new ArrayList<>();
        mathExpressionSymbols.addAll(getNewExpressions(mathArithmeticExpressionSymbol, mathArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions));
        mathExpressionSymbols.addAll(getNewExpressions(mathArithmeticExpressionSymbol, mathArithmeticExpressionSymbol.getRightExpression(), precedingExpressions));
        return mathExpressionSymbols;
    }

    public List<MathExpressionSymbol> getNewExpressions(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        List<MathExpressionSymbol> mathExpressionSymbols = new ArrayList<>();
        if (MathOptimizer.isArithmeticExpression(mathExpressionSymbol.getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            if (MathOptimizer.isArithmeticMatrixExpression(mathExpressionSymbol.getRealMathExpressionSymbol(), "*", precedingExpressions))
                mathExpressionSymbols.addAll(getNewExpressions((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol.getRealMathExpressionSymbol(), precedingExpressions));
            else
                mathExpressionSymbols.addAll(getNewExpressions((MathArithmeticExpressionSymbol) MathOptimizer.getCurrentAssignment(mathExpressionSymbol.getRealMathExpressionSymbol(), precedingExpressions), precedingExpressions));
        } else {
            mathExpressionSymbols.add(mathExpressionSymbol);
        }
        return mathExpressionSymbols;
    }

    /**
     * Adapted Dynamic Programming solution from https://en.wikipedia.org/wiki/Matrix_chain_multiplication
     *
     * @param mathExpressionSymbols
     * @return
     */
    public MathExpressionSymbol getMostEfficientExpression(List<MathExpressionSymbol> mathExpressionSymbols) {

        int dims[] = new int[mathExpressionSymbols.size() + 1];
        int n = dims.length - 1;
        int[][] m = new int[n][n];
        int[][] s = new int[n][n];
        //It is assumed, that the matrices fit together such that a valid multiplication can be performed.
        // Then, it is sufficient to store the row of first matrix and column of each following in the expression,
        // such that matrix_n = dim[n-1] x dim [n]

        //orig
        //dims[0] = MathDimensionCalculator.getMatrixColumns(mathExpressionSymbols.get(0), new ArrayList<MathExpressionSymbol>());
        dims[0] = MathDimensionCalculator.getMatrixRows(mathExpressionSymbols.get(0), new ArrayList<MathExpressionSymbol>());
        Log.info(dims[0] + mathExpressionSymbols.get(0).getTextualRepresentation(), "most effi");
        for (int i = 1; i <= mathExpressionSymbols.size(); ++i) {
            //orig
            //dims[i] = MathDimensionCalculator.getMatrixRows(mathExpressionSymbols.get(i - 1), new ArrayList<MathExpressionSymbol>());
            dims[i] = MathDimensionCalculator.getMatrixColumns(mathExpressionSymbols.get(i - 1), new ArrayList<MathExpressionSymbol>());
            Log.info(dims[i] + mathExpressionSymbols.get(i - 1).getTextualRepresentation(), "most effi");

        }

        //Debug: dims fulfills the statement above

        for (int lenMinusOne = 1; lenMinusOne < n; lenMinusOne++) {
            for (int i = 0; i < n - lenMinusOne; i++) {
                int j = i + lenMinusOne;
                m[i][j] = Integer.MAX_VALUE;
                for (int k = i; k < j; k++) {
                    int cost = m[i][k] + m[k + 1][j] + dims[i] * dims[k + 1] * dims[j + 1];
                    if (cost < m[i][j]) {
                        m[i][j] = cost;
                        s[i][j] = k;
                    }
                }
            }
        }

        //Debug: Cost calculation is exactly as in
        //https://en.wikipedia.org/wiki/Matrix_chain_multiplication

        /*for (MathExpressionSymbol mat : mathExpressionSymbols)
            Log.info(mat.getTextualRepresentation(), "Mat:");
        for (int i = 0; i < dims.length; ++i)
            Log.info("i:" + dims[i], "Dims:");
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < n; ++k) {
                Log.info("i:" + i + " k:" + k + " val: " + s[i][k], "S:");
            }

        for (int i = 0; i < n; ++i)
            for (int k = 0; k < n; ++k) {
                Log.info("i:" + i + " k:" + k + " val: " + m[i][k], "M:");
            }*/
        return buildOptimalExpression(s, mathExpressionSymbols);
    }

    //Algorithm is based on the algorithm from https://en.wikipedia.org/wiki/Matrix_chain_multiplication
    private Map<Integer, ResultOrder> resultMap = new HashMap<>();
    private Map<Integer, MathExpressionSymbol> stepMap = new HashMap<>();
    private int counter = 0;

    public MathExpressionSymbol buildOptimalExpression(int[][] s, List<MathExpressionSymbol> mathExpressionSymbols) {
        boolean[] inAResult = new boolean[s.length];
        resultMap.clear();
        counter = 0;
        buildOptimalExpression(s, 0, s.length - 1, inAResult);
        int lastIndex = 0;

        //Debug: Result Map looks correct

        for (int i = 0; resultMap.containsKey(i); ++i) {
            /*Log.info(i + "A_" + resultMap.get(i).matrixIndexLeft + resultMap.get(i).isResultLeft + " A_" +
                    resultMap.get(i).matrixIndexRight + resultMap.get(i).isResultRight, "Index:");*/
            MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol = new MathArithmeticExpressionSymbol();
            mathArithmeticExpressionSymbol.setMathOperator("*");
            ResultOrder resultOrder = resultMap.get(i);
            if (resultOrder.isResultLeft)
                mathArithmeticExpressionSymbol.setLeftExpression(stepMap.get(resultOrder.matrixIndexLeft));
            else
                mathArithmeticExpressionSymbol.setLeftExpression(mathExpressionSymbols.get(resultOrder.matrixIndexLeft));

            if (resultOrder.isResultRight)
                mathArithmeticExpressionSymbol.setRightExpression(stepMap.get(resultOrder.matrixIndexRight));
            else
                mathArithmeticExpressionSymbol.setRightExpression(mathExpressionSymbols.get(resultOrder.matrixIndexRight));
            stepMap.put(resultOrder.matrixIndexLeft, new MathParenthesisExpressionSymbol(mathArithmeticExpressionSymbol));
            stepMap.put(resultOrder.matrixIndexRight, new MathParenthesisExpressionSymbol(mathArithmeticExpressionSymbol));
            lastIndex = resultOrder.matrixIndexLeft;
        }

        //Debug: StepMap is correct!

        return stepMap.get(lastIndex);
    }

    private void buildOptimalExpression(int[][] s, int i, int j, boolean[] inAResult) {
        if (i != j) {
            buildOptimalExpression(s, i, s[i][j], inAResult);
            buildOptimalExpression(s, s[i][j] + 1, j, inAResult);
            String istr = inAResult[i] ? "_result " : " ";
            String jstr = inAResult[j] ? "_result " : " ";
            resultMap.put(counter, new ResultOrder(i, inAResult[i], j, inAResult[j]));
            ++counter;
            //System.out.println(" A_" + i + istr + "* A_" + j + jstr);
            inAResult[i] = true;
            inAResult[j] = true;
        }

    }

    public List<MathExpressionSymbol> getNewExpressions(MathMatrixArithmeticExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        List<MathExpressionSymbol> mathExpressionSymbols = new ArrayList<>();
        if (MathOptimizer.isArithmeticExpression(mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            if (MathOptimizer.isArithmeticMatrixExpression(mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions))
                mathExpressionSymbols.addAll(getNewExpressions((MathMatrixArithmeticExpressionSymbol) mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions));
            else
                mathExpressionSymbols.addAll(getNewExpressions((MathArithmeticExpressionSymbol) mathExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions));
        }
        return mathExpressionSymbols;
    }

}
