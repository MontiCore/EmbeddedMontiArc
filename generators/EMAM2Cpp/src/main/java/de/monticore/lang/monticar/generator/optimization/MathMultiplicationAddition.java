/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Rule that transforms AC+BC to (A+B)*C and CA+CB to C*(A+B)
 *
 */
public class MathMultiplicationAddition implements MathOptimizationRule {

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
        //TODO enable head optimization
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

    public void optimize(MathArithmeticExpressionSymbol mathArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(mathArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(mathArithmeticExpressionSymbol.getRightExpression(), precedingExpressions);

        if (mathArithmeticExpressionSymbol.getMathOperator().equals("+")) {
            optimizeMatrixAddition(mathArithmeticExpressionSymbol, precedingExpressions);
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

    public void optimize(MathMatrixNameExpressionSymbol matrixNameExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        //nothing to do in this rule
    }

    public void optimize(MathMatrixArithmeticExpressionSymbol matrixArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        optimize(matrixArithmeticExpressionSymbol.getLeftExpression(), precedingExpressions);
        optimize(matrixArithmeticExpressionSymbol.getRightExpression(), precedingExpressions);

        if (matrixArithmeticExpressionSymbol.getMathOperator().equals("+")) {
            optimizeMatrixAddition(matrixArithmeticExpressionSymbol, precedingExpressions);
        }

    }

    public void optimizeMatrixAddition(MathMatrixArithmeticExpressionSymbol matrixArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions) && MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            Log.debug("Optimizer case not handled", "Case not handled!");
        }
    }

    public void optimizeMatrixAddition(MathArithmeticExpressionSymbol matrixArithmeticExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), "*", precedingExpressions) && MathOptimizer.isArithmeticExpression(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), "*", precedingExpressions)) {
            MathExpressionSymbol leftLeftExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol leftRightExpressionSymbol = MathOptimizer.getRightExpressionSymbol(matrixArithmeticExpressionSymbol.getLeftExpression().getRealMathExpressionSymbol(), precedingExpressions);

            MathExpressionSymbol rightLeftExpressionSymbol = MathOptimizer.getLeftExpressionSymbol(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);
            MathExpressionSymbol rightRightExpressionSymbol = MathOptimizer.getRightExpressionSymbol(matrixArithmeticExpressionSymbol.getRightExpression().getRealMathExpressionSymbol(), precedingExpressions);

            Log.info(leftLeftExpressionSymbol.getTextualRepresentation(), "leftLeft");
            Log.info(leftRightExpressionSymbol.getTextualRepresentation(), "leftRight");

            Log.info(rightLeftExpressionSymbol.getTextualRepresentation(), "rightLeft");
            Log.info(rightRightExpressionSymbol.getTextualRepresentation(), "rightRight");
            //Log.error("pre");
            if (leftLeftExpressionSymbol.getTextualRepresentation().equals(rightLeftExpressionSymbol.getTextualRepresentation())) {
//                MathExpressionSymbol oldLeftExpression=matrixArithmeticExpressionSymbol.getLeftExpression();
//                MathExpressionSymbol oldRightExpression=matrixArithmeticExpressionSymbol.getRightExpression();

                matrixArithmeticExpressionSymbol.setMathOperator("*");
                matrixArithmeticExpressionSymbol.setLeftExpression(leftLeftExpressionSymbol);
                MathArithmeticExpressionSymbol newRightExpressionSymbol = new MathArithmeticExpressionSymbol();
                newRightExpressionSymbol.setMathOperator("+");
                newRightExpressionSymbol.setLeftExpression(leftRightExpressionSymbol);
                newRightExpressionSymbol.setRightExpression(rightRightExpressionSymbol);
                matrixArithmeticExpressionSymbol.setRightExpression(new MathParenthesisExpressionSymbol(newRightExpressionSymbol));
            } else if (leftRightExpressionSymbol.getTextualRepresentation().equals(rightRightExpressionSymbol.getTextualRepresentation())) {

                matrixArithmeticExpressionSymbol.setMathOperator("*");
                matrixArithmeticExpressionSymbol.setRightExpression(rightRightExpressionSymbol);
                MathArithmeticExpressionSymbol newLeftExpressionSymbol = new MathArithmeticExpressionSymbol();
                newLeftExpressionSymbol.setMathOperator("+");
                newLeftExpressionSymbol.setLeftExpression(leftLeftExpressionSymbol);
                newLeftExpressionSymbol.setRightExpression(rightLeftExpressionSymbol);
                matrixArithmeticExpressionSymbol.setLeftExpression(new MathParenthesisExpressionSymbol(newLeftExpressionSymbol));

            }


            /*for(Variable var:MathOptimizer.currentBluePrint.getMathInformationRegister().getVariables())
            Log.info(var.getTypeNameTargetLanguage()+" "+var.getName(),"Var:");
            Log.error("a");*/
        }
    }


}
