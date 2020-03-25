/* (c) https://github.com/MontiCore/monticore */
package de.monticore.numberunit._ast;

import de.monticore.numberunit._parser.NumberUnitParser;
import org.junit.Test;

import javax.measure.unit.Unit;
import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 */
public class ASTNumberWithUnitTest {

    private static NumberUnitParser parser = null;

    protected static NumberUnitParser getParser() {
        if (parser == null)
            parser = new NumberUnitParser();
        return parser;
    }

    protected static void setParser(NumberUnitParser parser) {
        ASTNumberWithUnitTest.parser = parser;
    }

    @Test
    public void isPlusInfinite() throws IOException {
        assertEquals(true, getASTFromString("oo").isPlusInfinite());
    }

    @Test
    public void isMinusInfinite() throws IOException {
        assertEquals(true, getASTFromString("-oo").isMinusInfinite());
    }

    @Test
    public void isComplexNumber() throws IOException {
        assertEquals(true, getASTFromString("1 + 1i").isComplexNumber());
    }

    @Test
    public void getNumber() throws IOException {
        assertEquals(42, getASTFromString("42").getNumber().get(), 0);
    }

    @Test
    public void setNumber() throws IOException {
        ASTNumberWithUnit ast = getASTFromString("0");
        ast.setNumber(1.0);
        assertEquals(1, ast.getNumber().get(), 0);
    }

    @Test
    public void getUnit() throws IOException {
        assertEquals("m", getASTFromString("1m").getUnit().toString());
    }

    @Test
    public void setUnit() throws IOException {
        ASTNumberWithUnit ast = getASTFromString("1");
        ast.setUnit(Unit.valueOf("m"));
        assertEquals("m", ast.getUnit().toString());
    }

    protected ASTNumberWithUnit getASTFromString(String in) throws IOException {
        ASTNumberWithUnit ast = getParser().parse_StringNumberWithUnit(in).orElse(null);
        assertNotNull(ast);
        return ast;
    }
}
