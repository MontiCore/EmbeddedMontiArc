/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAPortInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.monticar.semantics.Constants;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ComponentCall;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.EquationSystemFunction;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.ExplicitFunction;
import de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit.SemiExplicitForm;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

import static de.monticore.lang.monticar.semantics.loops.analyze.EquationSystemType.*;

public class AnalyzeEquationSystemType implements EMAMMathExpressionSymbolVisitor {

    private final Collection<String> linearFunctions = Arrays.asList("sum");

    public static EquationSystemType typeOf(EMAMEquationSymbol expressionSymbol, Collection<EMAMSymbolicVariableSymbol> variables) {
        AnalyzeEquationSystemType analyzeEquationSystemType = new AnalyzeEquationSystemType(variables.stream().map(v -> v.getName()).collect(Collectors.toSet()));
        analyzeEquationSystemType.handle(expressionSymbol);
        return analyzeEquationSystemType.getType();
    }

    public static EquationSystemType typeOf(Collection<EMAMEquationSymbol> equations, Collection<EMAMSymbolicVariableSymbol> variables) {
        if (equations.size() < variables.size())
            return Underspecified;
        if (equations.size() > variables.size())
            return Overspecified;
        EquationSystemType currentKind = Constant;
        for (EMAMEquationSymbol equation : equations) {
            EquationSystemType equationSystemType = AnalyzeEquationSystemType.typeOf(equation, variables);
            currentKind = EquationSystemTypeCombiner.combine(currentKind, equationSystemType);
        }
        return currentKind;
    }

    public static EquationSystemType typeOf(SemiExplicitForm semiExplicitForm) {
        EquationSystemType type = Constant;
        if (!semiExplicitForm.getF().isEmpty()) {
            type = ODE;
            if (semiExplicitForm.getG().isEmpty())
                return type;
        }

        Collection<EMAMSymbolicVariableSymbol> variables = new HashSet<>();
        variables.addAll(semiExplicitForm.getY());
        variables.addAll(semiExplicitForm.getZ());
        for (EquationSystemFunction equationSystemFunction : semiExplicitForm.getG()) {
            if (equationSystemFunction instanceof ComponentCall)
                return EquationSystemTypeCombiner.combine(type, NonLinear);
            else if (equationSystemFunction instanceof ExplicitFunction) {
                type = EquationSystemTypeCombiner.combine(type,
                        typeOf(((ExplicitFunction) equationSystemFunction).getEquation(), variables));
            }
        }

        return type;
    }

    public EquationSystemType getType() {
        return lastResult;
    }

    private EquationSystemType lastResult = Constant;
    private final Set<String> variables;

    public AnalyzeEquationSystemType(Set<String> variables) {
        this.variables = variables;
    }

    @Override
    public void traverse(MathArithmeticExpressionSymbol node) {
        lastResult = Constant;
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        EquationSystemType leftResult = lastResult;

        lastResult = Constant;
        if (node.getRightExpression() != null) handle(node.getRightExpression());
        EquationSystemType rightResult = lastResult;

        if (node.isAdditionOperator() || node.isSubtractionOperator())
            lastResult = EquationSystemTypeCombiner.combine(leftResult, rightResult);
        else if (leftResult != Constant && rightResult != Constant) // one side is just constant
            lastResult = EquationSystemTypeCombiner.combineKinds(NonLinear, leftResult, rightResult);
        else
            lastResult = EquationSystemTypeCombiner.combine(leftResult, rightResult);
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
            lastResult = Linear;
        else
            lastResult = Constant;
    }

    @Override
    public void traverse(MathNumberExpressionSymbol node) {
        lastResult = Constant;
    }

    @Override
    public void traverse(MathPreOperatorExpressionSymbol node) {
        // TODO
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(MathValueSymbol node) {
        // TODO
        notSupported(node);
    }

    @Override
    public void traverse(MathValueType node) {
        // TODO
        notSupported(node);
    }

    @Override
    public void traverse(MathMatrixAccessOperatorSymbol node) {
        EquationSystemType result = Constant;
        for (MathMatrixAccessSymbol access : node.getMathMatrixAccessSymbols()) {
            lastResult = Constant;
            handle(access);
            result = EquationSystemTypeCombiner.combine(result, lastResult);
        }
        lastResult = result;
    }

    @Override
    public void traverse(MathMatrixNameExpressionSymbol node) {
        lastResult = Constant;
        if (node.isMathMatrixAccessOperatorSymbolPresent())
            handle(node.getMathMatrixAccessOperatorSymbol());
        EquationSystemType innerKind = lastResult;

        if (isAccess(node))
            lastResult = innerKind;
        else if (isLinearFunction(node))
            lastResult = innerKind == Constant ? Constant : EquationSystemTypeCombiner.combine(Linear, innerKind);
        if (isIntegral(node) || isDerivative(node)) {
            switch (innerKind) {
                case Constant:
                    lastResult = Constant;
                    break;
                case Linear:
                    lastResult = ODE;
                    break;
                case NonLinear:
                    notSupported(node);
                    break;
                case Polynom:
                    notSupported(node);
                    break;
                case ODE:
                    notSupported(node);
                    break;
                case DAE:
                    notSupported(node);
                    break;
                default:
                    notSupported(node);
                    break;
            }
        } else
            lastResult = innerKind == Constant ? Constant : EquationSystemTypeCombiner.combine(NonLinear, innerKind);
    }

    private boolean isAccess(MathMatrixNameExpressionSymbol node) {
        Optional<EMAPortInstanceSymbol> port =
                node.getEnclosingScope().resolve(node.getNameToAccess(), EMAPortInstanceSymbol.KIND);
        Optional<EMAMSymbolicVariableSymbol> variable =
                node.getEnclosingScope().resolve(node.getNameToAccess(), EMAMSymbolicVariableSymbol.KIND);
        if (port.isPresent() && port.get().isPartOfPortArray())
            return true;
        else if (variable.isPresent() && variable.get().getType().getDimensions().size() > 0)
            return true;
        return false;
    }

    private boolean isLinearFunction(MathMatrixNameExpressionSymbol node) {
        return linearFunctions.contains(node.getNameToAccess());
    }

    private boolean isIntegral(MathMatrixNameExpressionSymbol node) {
        return Constants.integrateOperatorName.equals(node.getNameToAccess());
    }

    private boolean isDerivative(MathMatrixNameExpressionSymbol node) {
        return Constants.derivativeOperatorName.equals(node.getNameToAccess());
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
        lastResult = Constant;
        if (node.getLeftExpression() != null) handle(node.getLeftExpression());
        EquationSystemType leftResult = lastResult;

        lastResult = Constant;
        if (node.getRightExpression() != null) handle(node.getRightExpression());
        EquationSystemType rightResult = lastResult;

        if (node.isAdditionOperator() || node.isSubtractionOperator())
            lastResult = EquationSystemTypeCombiner.combine(leftResult, rightResult);
        else if (leftResult != Constant && rightResult != Constant) // one side is just constant
            lastResult = EquationSystemTypeCombiner.combineKinds(NonLinear, leftResult, rightResult);
        else
            lastResult = EquationSystemTypeCombiner.combine(leftResult, rightResult);
    }

    @Override
    public void traverse(EMAMEquationSymbol node) {
        lastResult = Constant;
        handle(node.getLeftExpression());
        EquationSystemType leftResult = lastResult;

        lastResult = Constant;
        handle(node.getRightExpression());
        EquationSystemType rightResult = lastResult;

        if (leftResult == ODE || leftResult == DAE)
            lastResult = leftResult;
        else if (rightResult == ODE || rightResult == DAE)
            lastResult = rightResult;
        else
            lastResult = EquationSystemTypeCombiner.combine(leftResult, rightResult);
    }

    @Override
    public void traverse(EMAMInitialGuessSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(EMAMInitialValueSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(MathParenthesisExpressionSymbol node) {
        EMAMMathExpressionSymbolVisitor.super.traverse(node);
    }

    @Override
    public void traverse(EMAMSpecificationSymbol node) {
        notSupported(node);
    }

    @Override
    public void traverse(EMAMSymbolicVariableSymbol node) {
        notSupported(node);
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
