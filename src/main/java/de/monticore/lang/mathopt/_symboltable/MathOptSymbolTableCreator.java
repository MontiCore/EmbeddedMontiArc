/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.commonexpressions._ast.ASTLessEqualExpression;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathForLoopExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
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

    public void endVisit(final ASTOptimizationVariableDeclarationExpression astExpression) {
        MathValueSymbol symbol = new MathValueSymbol(astExpression.getName());
        if (astExpression.getTypeOpt().isPresent())
            symbol.setType(MathValueType.convert(astExpression.getTypeOpt().get()));
        addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationForLoopExpression astExpression) {
        MathForLoopExpressionSymbol symbol = new MathForLoopExpressionSymbol();
        if (astExpression.getHead().getSymbolOpt().isPresent())
            symbol.setForLoopHead((MathForLoopHeadSymbol) astExpression.getHead().getSymbolOpt().get());
        for (ASTOptimizationConditionExpression astMathExpression : astExpression.getBodyList())
            if (astMathExpression.getSymbolOpt().isPresent())
                symbol.addForLoopBody((MathExpressionSymbol) astMathExpression.getSymbolOpt().get());
        addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationSimpleConditionExpression ast) {
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

    public void endVisit(final ASTOptimizationConditionExpression astExpression) {
        MathExpressionSymbol symbol = null;
        if (astExpression.getSimpleConditionOpt().isPresent() && astExpression.getSimpleConditionOpt().isPresent()) {
            symbol = (MathExpressionSymbol) astExpression.getSimpleCondition().getSymbolOpt().get();
        } else if (astExpression.getBoundedConditionOpt().isPresent() && astExpression.getBoundedCondition().getSymbolOpt().isPresent()) {
            symbol = (MathExpressionSymbol) astExpression.getBoundedCondition().getSymbolOpt().get();
        } else if (astExpression.getForLoopConditionOpt().isPresent()) {
            ASTOptimizationForLoopExpression loopExpr = astExpression.getForLoopConditionOpt().get();
            if (loopExpr.getSymbolOpt().isPresent()) {
                symbol = (MathExpressionSymbol) loopExpr.getSymbolOpt().get();
            }
        }
        if (symbol != null)
            addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationObjectiveFunction ast) {
        MathExpressionSymbol symbol;
        if (ast.getExpressionOpt().isPresent() && ast.getExpression().getSymbolOpt().isPresent()) {
            symbol = (MathExpressionSymbol) ast.getExpression().getSymbolOpt().get();
        } else if (ast.getStatementOpt().isPresent() && ast.getStatement().getSymbolOpt().isPresent()) {
            symbol = (MathExpressionSymbol) ast.getStatement().getSymbolOpt().get();
        } else {
            symbol = null;
            Log.error(String.format("Can not find symbol for %s", ast.toString()), ast.get_SourcePositionStart());
        }
        if (symbol != null) {
            addToScopeAndLinkWithNode(symbol, ast);
        }
    }

    public void endVisit(final ASTOptimizationExpression astMathOptimizationExpression) {
        MathOptimizationExpressionSymbol symbol = new MathOptimizationExpressionSymbol();
        symbol.setOptimizationType(astMathOptimizationExpression.getOptimizationType().toString());
        if (astMathOptimizationExpression.getOptimizationVariable().getSymbolOpt().isPresent()) {
            symbol.setOptimizationVariable((MathValueSymbol) astMathOptimizationExpression.getOptimizationVariable().getSymbolOpt().get());
        }
        if (astMathOptimizationExpression.getObjectiveFunction().getSymbolOpt().isPresent()) {
            symbol.setObjectiveExpression((MathExpressionSymbol) astMathOptimizationExpression.getObjectiveFunction().getSymbolOpt().get());
        }
        for (ASTOptimizationConditionExpression condition : astMathOptimizationExpression.getConstraintList()) {
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
        addToScopeAndLinkWithNode(symbol, astMathOptimizationExpression);
    }

}

