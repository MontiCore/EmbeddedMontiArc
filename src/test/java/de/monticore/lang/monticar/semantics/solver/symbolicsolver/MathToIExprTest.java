package de.monticore.lang.monticar.semantics.solver.symbolicsolver;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import org.junit.Test;
import org.matheclipse.core.interfaces.IExpr;

import java.io.IOException;
import java.io.StringReader;
import java.util.Optional;

import static org.junit.Assert.*;

public class MathToIExprTest {

    EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
    MathToIExpr converter = new MathToIExpr();

    @Test
    public void testConvert1() {
        ASTExpression expression = parseExpression("x_2=a_1-x_1");
        IExpr iast = converter.convert(expression);
        System.out.println(iast.toString());
    }

    @Test
    public void testConvert2() {
        ASTExpression expression = parseExpression("0=a_1-x_1-x_2");
        IExpr iast = converter.convert(expression);
        System.out.println(iast.toString());
    }

    private ASTExpression parseExpression(String s) {
        Optional<ASTExpression> expression = null;
        try {
            expression = parser.parseExpression(new StringReader(s));
        } catch (IOException e) {
            fail(e.getMessage());
        }

        if (!expression.isPresent())
            fail();

        return expression.get();
    }
}