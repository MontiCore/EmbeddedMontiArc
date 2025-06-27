/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.pp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathVisitor;
import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.prettyprint.IndentPrinter;

public class EmbeddedMontiArcExpressionPrettyPrinter implements EmbeddedMontiArcMathVisitor {

    private IndentPrinter printer;

    public EmbeddedMontiArcExpressionPrettyPrinter(IndentPrinter printer) {
        this.printer = printer;
    }

    private EmbeddedMontiArcMathVisitor realThis = this;

    @Override
    public void setRealThis(EmbeddedMontiArcMathVisitor realThis) {
        this.realThis = this;
    }

    @Override
    public EmbeddedMontiArcMathVisitor getRealThis() {
        return realThis;
    }

    @Override
    public void visit(ASTNumberExpression expr) {
        String val = "" + expr.getNumberWithUnit().getNumber().get().doubleValue();
        printer.print(val);
    }

    @Override
    public void visit(ASTNameExpression expr) {
        printer.print(expr.getName());
    }

}
