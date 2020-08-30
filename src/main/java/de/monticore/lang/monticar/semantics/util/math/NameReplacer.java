/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

public class NameReplacer implements MathExpressionSymbolVisitor {

    private final String oldName;
    private final String newName;

    public NameReplacer(String newName, String oldName) {
        this.newName = newName;
        this.oldName = oldName;
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
        if (node.getNameToResolveValue().equals(oldName))
            node.setNameToResolveValue(newName);
    }
}
