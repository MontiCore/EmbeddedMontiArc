/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

import java.util.Collection;

public class ContainsName implements MathExpressionSymbolVisitor {
    private boolean result = false;
    private Collection<String> names;

    public static boolean containsName(MathExpressionSymbol expression, Collection<String> names) {
        ContainsName instance = new ContainsName();
        instance.result = false;
        instance.names = names;

        instance.handle(expression);
        return instance.result;
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
        for (String name : names) {
            if (node.getNameToResolveValue().equals(name))
                result = true;
        }
    }
}
