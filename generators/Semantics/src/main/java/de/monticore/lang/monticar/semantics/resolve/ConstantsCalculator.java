/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.math._ast.ASTMathDottedNameExpression;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._visitor.MathVisitor;

import java.util.HashSet;
import java.util.Set;

public class ConstantsCalculator implements MathVisitor {
    private final Set<String> variables;
    private Set<String> constants = new HashSet<>();

    public ConstantsCalculator(Set<String> variables) {
        this.variables = variables;
    }

    public Set<String> getConstants() {
        return this.constants;
    }

    @Override
    public void visit(ASTNameExpression node) {
        if (!variables.contains(node.getName()))
            constants.add(node.getName());
    }

    @Override
    public void visit(ASTMathDottedNameExpression node) {
        String name = String.join(".", node.getNameList());
        if (!variables.contains(name))
            constants.add(name);
    }
}
