/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

import de.monticore.lang.math._ast.ASTMathFalseExpression;
import de.monticore.lang.math._ast.ASTMathTrueExpression;
import de.monticore.lang.math._symboltable.MathSymbolTableCreator;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.monticar.cnnarch.predefined.AllPredefinedVariables;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Deque;

public class ModifiedMathSymbolTableCreator extends MathSymbolTableCreator {

    public ModifiedMathSymbolTableCreator(ResolvingConfiguration resolverConfiguration, MutableScope enclosingScope) {
        super(resolverConfiguration, enclosingScope);
    }

    public ModifiedMathSymbolTableCreator(ResolvingConfiguration resolvingConfig, Deque<MutableScope> scopeStack) {
        super(resolvingConfig, scopeStack);
    }


    @Override
    public void endVisit(ASTMathTrueExpression node){
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(AllPredefinedVariables.TRUE_NAME);
        addToScopeAndLinkWithNode(symbol, node);
    }

    @Override
    public void endVisit(ASTMathFalseExpression node){
        MathNameExpressionSymbol symbol = new MathNameExpressionSymbol(AllPredefinedVariables.FALSE_NAME);
        addToScopeAndLinkWithNode(symbol, node);
    }
}
