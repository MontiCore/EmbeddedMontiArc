/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.pp;

import de.monticore.lang.math._ast.ASTNameExpression;
import de.monticore.lang.math._ast.ASTNumberExpression;
import de.monticore.lang.math._visitor.MathVisitor;
import de.monticore.prettyprint.IndentPrinter;

public class MathPrettyPrinter implements MathVisitor {

    public MathVisitor realThis;

    private IndentPrinter printer;

    public MathPrettyPrinter(IndentPrinter printer) {
        this.printer = printer;
        realThis = this;
    }

    @Override
    public void setRealThis(MathVisitor realThis) {
        this.realThis = this;
    }

    @Override
    public MathVisitor getRealThis() {
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
