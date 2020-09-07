/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.math;

import de.monticore.lang.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixNameExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

import java.util.Map;

public class NameReplacer extends MathExpressionSymbolVisitor {

    private final Map<String, String> nameMapping;

    public NameReplacer(Map<String, String> nameMapping) {
        this.nameMapping = nameMapping;
    }

    @Override
    public void visit(MathNameExpressionSymbol node) {
        if (nameMapping.containsKey(node.getNameToAccess()))
            node.setNameToResolveValue(nameMapping.get(node.getNameToAccess()));
    }

    @Override
    public void visit(MathAssignmentExpressionSymbol node) {
        if (nameMapping.containsKey(node.getNameOfMathValue()))
            node.setNameOfMathValue(nameMapping.get(node.getNameOfMathValue()));
    }

    @Override
    public void visit(MathMatrixNameExpressionSymbol node) {
        if (nameMapping.containsKey(node.getNameToAccess()))
            node.setNameToAccess(nameMapping.get(node.getNameToAccess()));
    }
}
