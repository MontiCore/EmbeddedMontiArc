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

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathForLoopExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.expression.MathValueType;
import de.monticore.lang.mathopt._ast.*;
import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.ImportStatement;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;
import de.monticore.types.types._ast.ASTImportStatement;
import de.se_rwth.commons.Names;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;

/**
 * Creates a hand written symbol table for MontiMathOpt
 * Assigns symbols to corresponding AST nodes.
 *
 * @author Christoph Richter
 */
public class MathOptSymbolTableCreator extends MathOptSymbolTableCreatorTOP {

    @Override
    public void visit(final ASTMathOptCompilationUnit compilationUnit) {
        Log.debug("Building Symboltable for Script: " + compilationUnit.getMathCompilationUnit().getMathScript().getName(),
                MathOptSymbolTableCreator.class.getSimpleName());

        // imports
        List<ImportStatement> imports = new ArrayList<>();
        for (ASTImportStatement astImportStatement : compilationUnit.getMathCompilationUnit().getImportStatementList()) {
            String qualifiedImport = Names.getQualifiedName(astImportStatement.getImportList());
            ImportStatement importStatement = new ImportStatement(qualifiedImport,
                    astImportStatement.isStar());
            imports.add(importStatement);
        }
        String package2 = "";
        if (compilationUnit.getMathCompilationUnit().isPresentPackage()) {
            package2 = Names.getQualifiedName(compilationUnit.getMathCompilationUnit().getPackage().getPartList());
        }
        ArtifactScope artifactScope = new ArtifactScope(
                Optional.empty(),
                package2,
                imports);
        //this.currentImports = imports;
        putOnStack(artifactScope);
    }

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, MutableScope enclosingScope) {
        super(resolvingConfig, enclosingScope);
    }

    public MathOptSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
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

    public void endVisit(final ASTOptimizationConditionExpression astExpression) {

        MathExpressionSymbol symbol = null;

        if (astExpression.getSimpleConditionOpt().isPresent()) {
            ASTOptimizationSimpleConditionExpression simpleExpr = astExpression.getSimpleConditionOpt().get();
            MathExpressionSymbol left = (MathExpressionSymbol) simpleExpr.getLeft().getSymbolOpt().orElse(null);
            MathExpressionSymbol right = (MathExpressionSymbol) simpleExpr.getRight().getSymbolOpt().orElse(null);
            String operator = simpleExpr.getOperator().getOperator();
            if ((!operator.isEmpty()) && (left != null) && (right != null)) {
                symbol = new MathOptimizationConditionSymbol(left, operator, right);
            }
        } else if (astExpression.getBoundedConditionOpt().isPresent()) {
            ASTOptimizationBoundsConditionExpression boundExpr = astExpression.getBoundedConditionOpt().get();
            MathExpressionSymbol lower = (MathExpressionSymbol) boundExpr.getLower().getSymbolOpt().orElse(null);
            MathExpressionSymbol expr = (MathExpressionSymbol) boundExpr.getExpr().getSymbolOpt().orElse(null);
            MathExpressionSymbol upper = (MathExpressionSymbol) boundExpr.getUpper().getSymbolOpt().orElse(null);
            if ((lower != null) && (expr != null) && (upper != null)) {
                symbol = new MathOptimizationConditionSymbol(lower, expr, upper);
            }
        } else if (astExpression.getForLoopConditionOpt().isPresent()) {
            ASTOptimizationForLoopExpression loopExpr = astExpression.getForLoopConditionOpt().get();
            if (loopExpr.getSymbolOpt().isPresent()) {
                symbol = (MathExpressionSymbol) loopExpr.getSymbolOpt().get();
            }
        }

        if (symbol != null)
            addToScopeAndLinkWithNode(symbol, astExpression);
    }

    public void endVisit(final ASTOptimizationExpression astMathOptimizationExpression) {
        MathOptimizationExpressionSymbol symbol = new MathOptimizationExpressionSymbol();
        symbol.setOptimizationType(astMathOptimizationExpression.getOptimizationType().toString());
        if (astMathOptimizationExpression.getOptimizationVariable().getSymbolOpt().isPresent()) {
            symbol.setOptimizationVariable((MathValueSymbol) astMathOptimizationExpression.getOptimizationVariable().getSymbolOpt().get());
        }
        ASTExpression objective = astMathOptimizationExpression.getObjectiveFunction();
        if (objective.getSymbolOpt().isPresent()) {
            symbol.setObjectiveExpression((MathExpressionSymbol) objective.getSymbolOpt().get());
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

