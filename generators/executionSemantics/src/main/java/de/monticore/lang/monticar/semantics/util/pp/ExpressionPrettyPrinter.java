/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.util.pp;

import de.monticore.expressions.prettyprint.*;
import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._visitor.EmbeddedMontiArcMathDelegatorVisitor;
import de.monticore.prettyprint.IndentPrinter;

public class ExpressionPrettyPrinter extends EmbeddedMontiArcMathDelegatorVisitor {

    private IndentPrinter printer;

    private AssignmentExpressionsPrettyPrinter assignmentExpressionsPrettyPrinter;
    private CommonExpressionsPrettyPrinter commonExpressionsPrettyPrinter;
    private JavaClassExpressionsPrettyPrinter javaClassExpressionsPrettyPrinter;
    private MCExpressionsPrettyPrinter mcExpressionsPrettyPrinter;
    private OCLExpressionsPrettyPrinter oclExpressionsPrettyPrinter;
    private SetExpressionsPrettyPrinter setExpressionsPrettyPrinter;
    private ShiftExpressionsPrettyPrinter shiftExpressionsPrettyPrinter;
    private EmbeddedMontiArcExpressionPrettyPrinter embeddedMontiArcExpressionPrettyPrinter;
    private MathPrettyPrinter mathPrettyPrinter;

    public ExpressionPrettyPrinter(IndentPrinter printer) {
        this.printer = printer;
        assignmentExpressionsPrettyPrinter = new AssignmentExpressionsPrettyPrinter(printer);
        commonExpressionsPrettyPrinter = new CommonExpressionsPrettyPrinter(printer);
        javaClassExpressionsPrettyPrinter = new JavaClassExpressionsPrettyPrinter(printer);
        mcExpressionsPrettyPrinter = new MCExpressionsPrettyPrinter(printer);
        oclExpressionsPrettyPrinter = new OCLExpressionsPrettyPrinter(printer);
        setExpressionsPrettyPrinter = new SetExpressionsPrettyPrinter(printer);
        shiftExpressionsPrettyPrinter = new ShiftExpressionsPrettyPrinter(printer);
        embeddedMontiArcExpressionPrettyPrinter = new EmbeddedMontiArcExpressionPrettyPrinter(printer);
        mathPrettyPrinter = new MathPrettyPrinter(printer);

        setCommonExpressionsVisitor(commonExpressionsPrettyPrinter);
        setAssignmentExpressionsVisitor(assignmentExpressionsPrettyPrinter);
        setEmbeddedMontiArcMathVisitor(embeddedMontiArcExpressionPrettyPrinter);
        setMathVisitor(mathPrettyPrinter);
    }

    public static String prettyPrint(ASTExpression node) {
        ExpressionPrettyPrinter prettyPrinter = new ExpressionPrettyPrinter(new IndentPrinter());
        node.accept(prettyPrinter);
        return prettyPrinter.getPrinter().getContent();
    }

    public IndentPrinter getPrinter() {
        return printer;
    }
}
