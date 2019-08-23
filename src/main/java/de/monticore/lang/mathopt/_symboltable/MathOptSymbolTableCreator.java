/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.commonexpressions._ast.ASTLessEqualExpression;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.mathopt._ast.*;
import de.monticore.lang.mathopt._visitor.MathOptVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.Deque;

/**
 * Creates a hand written symbol table for MontiMathOpt
 * Assigns symbols to corresponding AST nodes.
 *
 * @author Christoph Richter
 */
public class MathOptSymbolTableCreator extends MathSymbolTableCreator implements MathOptVisitor {

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    @Override
    public MathOptVisitor getRealThis() {
        return this;
    }

    @Override
    public void setRealThis(MathOptVisitor realThis) {

    }

    public void endVisit(final ASTOptimizationObjectiveValue node) {
        MathValueSymbol symbol = new MathValueSymbol(node.getName());
        if (node.getType() != null) {
            MathValueType type = new MathValueType();
            type.setType(node.getType());
            symbol.setType(type);
        }
        addToScopeAndLinkWithNode(symbol, node);
    }

    public void endVisit(final ASTOptimizationVariableDeclaration astExpression) {
        MathValueSymbol symbol = new MathValueSymbol(astExpression.getName());
        if (astExpression.getTypeOpt().isPresent())
            symbol.setType(MathValueType.convert(astExpression.getTypeOpt().get()));
        addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationForLoop astExpression) {
        MathForLoopExpressionSymbol symbol = new MathForLoopExpressionSymbol();
        if (astExpression.getHead().getSymbolOpt().isPresent())
            symbol.setForLoopHead((MathForLoopHeadSymbol) astExpression.getHead().getSymbolOpt().get());
        for (ASTOptimizationCondition astMathExpression : astExpression.getBodyList())
            if (astMathExpression.getSymbolOpt().isPresent())
                symbol.addForLoopBody((MathExpressionSymbol) astMathExpression.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationSimpleCondition ast) {
        MathExpressionSymbol symbol = null;
        if (ast.getLeft() instanceof ASTLessEqualExpression) {
            ASTLessEqualExpression leq = (ASTLessEqualExpression) ast.getLeft();
            MathExpressionSymbol lower = (MathExpressionSymbol) leq.getLeftExpression().getSymbolOpt().orElse(null);
            MathExpressionSymbol expr = (MathExpressionSymbol) leq.getRightExpression().getSymbolOpt().orElse(null);
            MathExpressionSymbol upper = (MathExpressionSymbol) ast.getRight().getSymbolOpt().orElse(null);
            if ((lower != null) && (expr != null) && (upper != null)) {
                symbol = new MathOptimizationConditionSymbol(lower, expr, upper);
            }
        } else {
            MathExpressionSymbol left = (MathExpressionSymbol) ast.getLeft().getSymbolOpt().orElse(null);
            MathExpressionSymbol right = (MathExpressionSymbol) ast.getRight().getSymbolOpt().orElse(null);
            String operator = ast.getOperator().getOperator();
            if ((!operator.isEmpty()) && (left != null) && (right != null)) {
                symbol = new MathOptimizationConditionSymbol(left, operator, right);
            }
        }
        if (symbol != null)
            addToScopeAndLinkWithNode(symbol, ast);
    }

    public void endVisit(final ASTOptimizationCondition astExpression) {
        MathExpressionSymbol symbol = null;
        if (astExpression.getSimpleConditionOpt().isPresent() && astExpression.getSimpleConditionOpt().isPresent()) {
            symbol = (MathExpressionSymbol) astExpression.getSimpleCondition().getSymbolOpt().get();
        } else if (astExpression.getBoundedConditionOpt().isPresent() && astExpression.getBoundedCondition().getSymbolOpt().isPresent()) {
            symbol = (MathExpressionSymbol) astExpression.getBoundedCondition().getSymbolOpt().get();
        } else if (astExpression.getForLoopConditionOpt().isPresent()) {
            ASTOptimizationForLoop loopExpr = astExpression.getForLoopConditionOpt().get();
            if (loopExpr.getSymbolOpt().isPresent()) {
                symbol = (MathExpressionSymbol) loopExpr.getSymbolOpt().get();
            }
        }
        if (symbol != null)
            addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationStatement astMathOptimizationStatement) {
        MathOptimizationStatementSymbol symbol = new MathOptimizationStatementSymbol();
        symbol.setOptimizationType(astMathOptimizationStatement.getOptimizationType().toString());
        if (astMathOptimizationStatement.getObjectiveValueOpt().isPresent()) {
            symbol.setObjectiveValue((MathValueSymbol) astMathOptimizationStatement.getObjectiveValue().getSymbolOpt().get());
        }
        if (astMathOptimizationStatement.getOptimizationVariable().getSymbolOpt().isPresent()) {
            symbol.setOptimizationVariable((MathValueSymbol) astMathOptimizationStatement.getOptimizationVariable().getSymbolOpt().get());
        }
        if (astMathOptimizationStatement.getObjectiveFunction().getSymbolOpt().isPresent()) {
            symbol.setObjectiveExpression((MathExpressionSymbol) astMathOptimizationStatement.getObjectiveFunction().getSymbolOpt().get());
        }
        for (ASTOptimizationCondition condition : astMathOptimizationStatement.getConstraintList()) {
            if (condition.getSymbolOpt().isPresent()) {
                MathExpressionSymbol conditionSymbol = (MathExpressionSymbol) condition.getSymbolOpt().get();
                if (conditionSymbol instanceof MathOptimizationConditionSymbol) {
                    ((MathOptimizationConditionSymbol) conditionSymbol).resolveBoundedExpressionToOptimizationVariable(symbol.getOptimizationVariable());
                } else if (conditionSymbol instanceof MathForLoopExpressionSymbol) {
                    for (MathExpressionSymbol sym : ((MathForLoopExpressionSymbol) conditionSymbol).getForLoopBody())
                        if (sym instanceof MathOptimizationConditionSymbol)
                            ((MathOptimizationConditionSymbol) sym).resolveBoundedExpressionToOptimizationVariable(symbol.getOptimizationVariable());
                }
                symbol.getSubjectToExpressions().add(conditionSymbol);
            }
        }
        addToScopeAndLinkWithNode(symbol, astMathOptimizationStatement);
    }
}

