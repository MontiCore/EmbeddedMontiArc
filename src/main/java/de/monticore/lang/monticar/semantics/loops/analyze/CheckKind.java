/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.se_rwth.commons.logging.Log;

import java.util.Set;

public class CheckKind extends MathExpressionSymbolVisitor {

    public LoopKind getKind() {
        return lastResult;
    }

    private LoopKind lastResult = LoopKind.Default;
    private final Set<String> variables;

    public CheckKind(Set<String> variables) {
        this.variables = variables;
    }

    @Override
    public void traverse(MathArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        LoopKind leftResult = lastResult;
        if (node.getRightExpression() != null) handle(node.getRightExpression());
        LoopKind rightResult = lastResult;
        if (node.isAdditionOperator() || node.isSubtractionOperator())
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
        else
            if (leftResult != LoopKind.Default && rightResult != LoopKind.Default)
                lastResult = LoopKindHelper.combineKinds(LoopKind.NonLinear, leftResult, rightResult);
            else
                lastResult = LoopKindHelper.combine(leftResult, rightResult);
    }

    @Override
    public void traverse(MathAssignmentExpressionSymbol node) {
        if (node.getMathMatrixAccessOperatorSymbol() != null) handle(node.getMathMatrixAccessOperatorSymbol());
        if (node.getExpressionSymbol() != null) handle(node.getExpressionSymbol());
    }

    @Override
    public void traverse(MathBooleanExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathCompareExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathConditionalExpressionsSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathConditionalExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathForLoopHeadSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathForLoopExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathNameExpressionSymbol node) {
        if (variables.contains(node.getNameToResolveValue()))
            lastResult = LoopKind.Linear;
    }

    @Override
    public void traverse(MathNumberExpressionSymbol node) {
        lastResult = LoopKind.Default;
    }

    @Override
    public void traverse(MathParenthesisExpressionSymbol node) {
        super.traverse(node);
    }

    @Override
    public void traverse(MathPreOperatorExpressionSymbol node) {
        // TODO
        handle(node.getMathExpressionSymbol());
    }

    @Override
    public void traverse(MathValueSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathValueType node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixAccessOperatorSymbol node) {
        // TODO
        if (node.getMathMatrixNameExpressionSymbol() != null) handle(node.getMathMatrixNameExpressionSymbol());
        for (MathMatrixAccessSymbol mathMatrixAccessSymbol : node.getMathMatrixAccessSymbols()) {
            handle(mathMatrixAccessSymbol);
        }
    }

    @Override
    public void traverse(MathMatrixNameExpressionSymbol node) {
        // TODO maybe is not a function?
        if (node.isMathMatrixAccessOperatorSymbolPresent()) handle(node.getMathMatrixAccessOperatorSymbol());
        if (lastResult != LoopKind.Default) {
            if (isIntegral(node) || isDerivative(node))
                lastResult = LoopKindHelper.combine(LoopKind.LinearDifferencial, lastResult);
        }
    }

    // TODO add more
    private final String derivativeName = "Derivative";
    private final String integralName = "Integral";

    private boolean isFunction(MathMatrixNameExpressionSymbol node) {
        // TODO
        return true;
    }

    private boolean isIntegral(MathMatrixNameExpressionSymbol node) {
        return integralName.equals(node.getNameToAccess());

    }

    private boolean isDerivative(MathMatrixNameExpressionSymbol node) {
        return derivativeName.equals(node.getNameToAccess());
    }

    @Override
    public void traverse(MathMatrixVectorExpressionSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixArithmeticValueSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixAccessSymbol node) {
        if (node.getMathExpressionSymbol().isPresent()) handle(node.getMathExpressionSymbol().get());
    }

    @Override
    public void traverse(MathMatrixArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        LoopKind leftResult = lastResult;
        if (node.getRightExpression() != null) handle(node.getRightExpression());
        LoopKind rightResult = lastResult;
        if (node.isAdditionOperator() || node.isSubtractionOperator())
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
        else
        if (leftResult != LoopKind.Default && rightResult != LoopKind.Default)
            lastResult = LoopKindHelper.combineKinds(LoopKind.NonLinear, leftResult, rightResult);
        else
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
    }

    private void notSupported(MathExpressionSymbol node) {
        Log.error("0xE0891240 Not supported MathExpressionSymbol: " + node.getTextualRepresentation());
    }
}
