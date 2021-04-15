/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._symboltable;

import de.monticore.commonexpressions._ast.ASTLessEqualExpression;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
import de.monticore.lang.math._symboltable.*;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.MathMatrixVectorExpressionSymbol;
import de.monticore.lang.mathopt._ast.*;
import de.monticore.lang.mathopt._parser.MathOptAntlrParser;
import de.monticore.lang.mathopt._visitor.MathOptVisitor;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

/**
 * Creates a hand written symbol table for MontiMathOpt
 * Assigns symbols to corresponding AST nodes.
 *
 */
public class MathOptSymbolTableCreator extends MathSymbolTableCreator implements MathOptVisitor {

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }

    private MathOptVisitor realThis = this;

    @Override
    public MathOptVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void setRealThis(MathOptVisitor realThis) {
        this.realThis = realThis;
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
        //Minimize or Maximize
        symbol.setOptimizationType(astMathOptimizationStatement.getOptimizationType().toString());
        if (astMathOptimizationStatement.getObjectiveValueOpt().isPresent()) {
            symbol.setObjectiveValue((MathValueSymbol) astMathOptimizationStatement.getObjectiveValue().getSymbolOpt().get());
        }
        //Optional stepsize definition( e.g. <n=1:10>)
        if (astMathOptimizationStatement.isPresentStepSize()) {
            ASTExpression x = astMathOptimizationStatement.getStepSizeOpt().get();
            MathExpressionSymbol temp = (MathExpressionSymbol) x.getSymbolOpt().get();
            symbol.setStepSizeExpression(temp);
        }
        //Optimization variables
        List<ASTOptimizationVariableDeclaration> optVarDecList = astMathOptimizationStatement.getOptimizationVariableList();
        List<MathValueSymbol> optVariables = new ArrayList<>();
        for (ASTOptimizationVariableDeclaration varDec : optVarDecList) {
            MathValueSymbol temp = (MathValueSymbol) varDec.getSymbolOpt().get();
            if(varDec.getInitializationOpt().isPresent()) {
                temp.setValue((MathExpressionSymbol) varDec.getInitializationOpt().get());
            }
            MathNumberExpressionSymbol newDim = new MathNumberExpressionSymbol();
            if(temp.getType().getDimensions().isEmpty() && astMathOptimizationStatement.isPresentStepSize()) {
                if(symbol.getStepSizeExpression() instanceof MathAssignmentExpressionSymbol) {
                    MathAssignmentExpressionSymbol assignExpression = (MathAssignmentExpressionSymbol) symbol.getStepSizeExpression();
                    if(assignExpression.getExpressionSymbol() instanceof MathMatrixVectorExpressionSymbol) {
                        MathMatrixVectorExpressionSymbol assignChildExpression = (MathMatrixVectorExpressionSymbol) assignExpression.getExpressionSymbol();
                        if(assignChildExpression.getEnd() instanceof MathNumberExpressionSymbol) {
                            MathNumberExpressionSymbol endExpression = (MathNumberExpressionSymbol) assignChildExpression.getEnd();
                            newDim.setValue(endExpression.getValue());
                            temp.getType().addDimension(newDim);
                        }
                    }
                }
            }
            optVariables.add(temp);
        }
        symbol.setOptimizationVariables(optVariables);
        //Independent variables
        List<ASTMathAssignmentDeclarationStatement> indVarDecList = astMathOptimizationStatement.getIndependentDeclarationList();
        List<MathValueSymbol> indVariables = new ArrayList<>();
        for (ASTMathAssignmentDeclarationStatement varDec : indVarDecList) {
            indVariables.add((MathValueSymbol) varDec.getSymbolOpt().get());
        }
        symbol.setIndependentVariables(indVariables);
        //Objective function
        if (astMathOptimizationStatement.getObjectiveFunction().getSymbolOpt().isPresent()) {
            symbol.setObjectiveExpression((MathExpressionSymbol) astMathOptimizationStatement.getObjectiveFunction().getSymbolOpt().get());
        }
        //Constraints
        for (ASTOptimizationCondition condition : astMathOptimizationStatement.getConstraintList()) {
            if (condition.getSymbolOpt().isPresent()) {
                MathExpressionSymbol conditionSymbol = (MathExpressionSymbol) condition.getSymbolOpt().get();

                if (conditionSymbol instanceof MathOptimizationConditionSymbol) {
                    ((MathOptimizationConditionSymbol) conditionSymbol).resolveBoundedExpressionToOptimizationVariable(symbol.getOptimizationVariables());
                    symbol.getConstraints().add((MathOptimizationConditionSymbol) conditionSymbol);
                } else if (conditionSymbol instanceof MathForLoopExpressionSymbol) {
                    for (MathExpressionSymbol sym : ((MathForLoopExpressionSymbol) conditionSymbol).getForLoopBody())
                        if (sym instanceof MathOptimizationConditionSymbol) {
                            ((MathOptimizationConditionSymbol) sym).resolveBoundedExpressionToOptimizationVariable(symbol.getOptimizationVariables());
                           }
                }
                symbol.getSubjectToExpressions().add(conditionSymbol);
            }
        }
        addToScopeAndLinkWithNode(symbol, astMathOptimizationStatement);
    }
}

