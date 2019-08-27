/* (c) https://github.com/MontiCore/monticore */
package de.monticore.numberunit._ast;

import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._parser.Types2Parser;
import org.junit.Test;

import java.io.IOException;

import static org.junit.Assert.*;

public class ASTElementTypeTest {

    private static Types2Parser parser = null;

    protected static Types2Parser getParser() {
        if (parser == null)
            parser = new Types2Parser();
        return parser;
    }

    @Test
    public void rationalWithRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("Q(0:1)");
        assertTrue(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void rationalWithoutRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("Q");
        assertTrue(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void rationalOnlyRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("(0:0.1:1)");
        assertTrue(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void wholeNumberWithRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("Z(0:1:10)");
        assertTrue(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void wholeNumberWithoutRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("Z");
        assertTrue(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void wholeNumberSimpleRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("(0:10)");
        assertTrue(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    @Test
    public void wholeNumberStepRange() throws IOException {
        ASTElementType rationalWithRange = getASTFromString("(0:2:10)");
        assertTrue(rationalWithRange.isWholeNumber());
        assertFalse(rationalWithRange.isRational());
        assertFalse(rationalWithRange.isBoolean());
        assertFalse(rationalWithRange.isComplex());
        assertFalse(rationalWithRange.isNaturalNumber());
    }

    protected ASTElementType getASTFromString(String in) throws IOException {
        ASTElementType ast = getParser().parse_StringElementType(in).orElse(null);
        assertNotNull(ast);
        return ast;
    }
}
