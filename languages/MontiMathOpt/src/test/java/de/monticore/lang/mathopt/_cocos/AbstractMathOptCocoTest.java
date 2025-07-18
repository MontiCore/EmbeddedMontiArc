/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.mathopt._cocos;

import de.monticore.lang.mathopt.OptimizationModelHelper;
import de.monticore.lang.mathopt._ast.ASTMathOptNode;
import de.monticore.lang.mathopt._parser.MathOptParser;
import de.monticore.lang.mathopt._symboltable.MathOptSymbolTableCreator;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.MutableScope;

import java.util.Optional;

/**
 * Abstract base class to create coco tests. Provides methods to generate an AST node from string and assign symbols to it
 *
 */
public abstract class AbstractMathOptCocoTest {

    private MathOptParser parser = new MathOptParser();

    protected MathOptParser getParser() {
        return parser;
    }

    protected void initializeSymbol(ASTMathOptNode astNode) {
        Optional<MathOptSymbolTableCreator> stc = OptimizationModelHelper.getInstance().getSymbolTableCreator(astNode, Optional.empty());
    }

}
