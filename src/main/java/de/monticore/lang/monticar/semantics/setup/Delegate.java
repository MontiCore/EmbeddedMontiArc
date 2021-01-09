/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.setup;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

public class Delegate {

    public static <T extends MathExpressionSymbol> T copyMathExpressionSymbol(T mathExpressionSymbol) {
        return CopyEMAMMathExpressionSymbol.copy(mathExpressionSymbol);
    }

    public static MathStatementsSymbol copyMathExpressionSymbol(MathStatementsSymbol mathStatementsSymbol) {
        return CopyEMAMMathExpressionSymbol.copy(mathStatementsSymbol);
    }

}
