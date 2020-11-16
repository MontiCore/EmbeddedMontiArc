/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.semantics.Options;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class CheckKind implements EMAMMathExpressionSymbolVisitor {

    public static LoopKind kindOf(MathExpressionSymbol expressionSymbol, Set<String> variables) {
        CheckKind checkKind = new CheckKind(variables);
        checkKind.handle(expressionSymbol);
        return checkKind.getKind();
    }

    public static LoopKind kindOf(EMAMEquationSymbol expressionSymbol, Collection<EMAMSymbolicVariableSymbol> variables) {
        CheckKind checkKind = new CheckKind(variables.stream().map(v -> v.getName()).collect(Collectors.toSet()));
        checkKind.handle(expressionSymbol);
        return checkKind.getKind();
    }

    public static LoopKind kindOf(Collection<EMAMEquationSymbol> equations, Collection<EMAMSymbolicVariableSymbol> variables) {
        LoopKind currentKind = LoopKind.Default;
        for (EMAMEquationSymbol equation : equations) {
            LoopKind loopKind = CheckKind.kindOf(equation, variables);
            currentKind = LoopKindHelper.combine(currentKind, loopKind);
        }
        return currentKind;
    }

    public static LoopKind kindOf(Collection<EMAMEquationSymbol> equations, Set<String> variables) {
        return kindOf(equations,
                variables.stream().map(v -> new EMAMSymbolicVariableSymbol(v)).collect(Collectors.toSet()));
    }

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
        else if (leftResult != LoopKind.Default && rightResult != LoopKind.Default)
            lastResult = LoopKindHelper.combineKinds(LoopKind.NonLinear, leftResult, rightResult);
        else
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
    }

    @Override
    public void traverse(MathAssignmentExpressionSymbol node) {
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
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
            lastResult = LoopKindHelper.combine(lastResult, LoopKind.Linear);
    }

    @Override
    public void traverse(MathNumberExpressionSymbol node) {
        lastResult = LoopKindHelper.combine(lastResult, LoopKind.Linear);
    }

    @Override
    public void traverse(MathPreOperatorExpressionSymbol node) {
        // TODO
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
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
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(MathMatrixNameExpressionSymbol node) {
        // TODO maybe is not a function?
        if (node.isMathMatrixAccessOperatorSymbolPresent()) handle(node.getMathMatrixAccessOperatorSymbol());
        if (isIntegral(node) || isDerivative(node))
            lastResult = LoopKindHelper.combine(LoopKind.LinearDifferencial, lastResult);

    }

    private boolean isFunction(MathMatrixNameExpressionSymbol node) {
        // TODO
        return true;
    }

    private boolean isIntegral(MathMatrixNameExpressionSymbol node) {
        return Options.integrateOperatorName.equals(node.getNameToAccess());

    }

    private boolean isDerivative(MathMatrixNameExpressionSymbol node) {
        return Options.derivativeOperatorName.equals(node.getNameToAccess());
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
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(MathMatrixArithmeticExpressionSymbol node) {
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        LoopKind leftResult = lastResult;
        if (node.getRightExpression() != null) handle(node.getRightExpression());
        LoopKind rightResult = lastResult;
        if (node.isAdditionOperator() || node.isSubtractionOperator())
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
        else if (leftResult != LoopKind.Default && rightResult != LoopKind.Default)
            lastResult = LoopKindHelper.combineKinds(LoopKind.NonLinear, leftResult, rightResult);
        else
            lastResult = LoopKindHelper.combine(leftResult, rightResult);
    }

    @Override
    public void traverse(EMAMEquationSymbol node) {
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(EMAMInitialGuessSymbol node) {

    }

    @Override
    public void traverse(EMAMInitialValueSymbol node) {

    }

    @Override
    public void traverse(MathParenthesisExpressionSymbol node) {
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(EMAMSpecificationSymbol node) {
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(EMAMSymbolicVariableSymbol node) {

    }

    @Override
    public void traverse(MathOptimizationStatementSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathOptimizationConditionSymbol node) {
        notSupported(node);
    }

    private Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

    @Override
    public Set<MathExpressionSymbol> getVisitedSymbols() {
        return visitedSymbols;
    }

    private void notSupported(MathExpressionSymbol node) {
        Log.error("0xE0891240 Not supported MathExpressionSymbol: " + node.getTextualRepresentation());
    }
}
