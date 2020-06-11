/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.analyze;

import de.monticore.lang.math._ast.ASTMathDottedNameExpression;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._visitor.MathVisitor;

import java.util.LinkedList;
import java.util.List;

public class NameReplacer implements MathVisitor {
    private final String replacement;
    private final String toReplace;

    NameReplacer(String toReplace, String replacement) {
        this.toReplace = toReplace;
        this.replacement = replacement;
    }

    @Override
    public void visit(ASTNameExpression node) {
        if (node.getName().equals(toReplace))
            node.setName(replacement);
    }

    @Override
    public void visit(ASTMathDottedNameExpression node) {
        String name = String.join(".", node.getNameList());
        if (name.equals(toReplace)) {
            List<String> nameList = new LinkedList<>();
            nameList.add(replacement);
            node.setNameList(nameList);
        }
    }
}
