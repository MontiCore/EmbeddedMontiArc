/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math;

import de.monticore.assignmentexpressions._ast.ASTAssignmentExpression;
import de.monticore.lang.math._ast.ASTMathAssignmentDeclarationStatement;
import de.monticore.lang.math._matrixprops.MatrixProperties;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.Assert.*;

/**
 * Created by Philipp Goerick on 26.07.2017.
 */

public class MatrixPropertiesTest extends MathSymbolTableCreatorTest {

    private Scope symTab;
    @Before
    public void setUp() {
        symTab = createSymTab("src/test/resources");
    }

    /**
     * checks if m1 is positive definite and invertible
     */
    @Test
    public void posDefTest() {
        //Q^{3,3} m1 = [2 -1 0;-1 2 -1;0 -1 2];
        final MathValueSymbol m1 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m1", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m1);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m1.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(symbol.getMatrixProperties().contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.PosDef));
        assertTrue(properties.contains(MatrixProperties.Invertible));
        assertFalse(properties.contains(MatrixProperties.NegDef));
        assertFalse(properties.contains(MatrixProperties.Diag));
    }

    /**
     * checks if m2 is negative definite and invertible
     */
    @Test
    public void negDefTest() {
        //Q^{3,3} m2 = [-5 0 0;0 -4 1;0 1 -4];
        final MathValueSymbol m2 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m2", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m2);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m2.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.NegDef));
        assertTrue(properties.contains(MatrixProperties.Invertible));
        assertFalse(properties.contains(MatrixProperties.PosDef));
        assertFalse(properties.contains(MatrixProperties.Diag));
    }

    /**
     * checks if m3 is diagonal and invertible
     */
    @Test
    public void diagTest(){
        //Q^{3,3} m3 = [2 0 0;0 2 0;0 0 2];
        final MathValueSymbol m3 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m3", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m3);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m3.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertTrue(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m4 is diagonal and not invertible
     */
    @Test
    public void notInvertibleTest(){
        //Q^{3,3} m4 = [2 0 0;0 0 0;0 0 2];
        final MathValueSymbol m4 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m4", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m4);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m4.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));
        assertFalse(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m5 is skew-hermitian and invertible (not possible until complex matrix support)
     */
    @Ignore @Test
    public void skewHermitianTest() {
        //C^{3,3} m5 = [i -1+2i 1;1+2i 0 -2-i;-1 -2-i -2i];
        final MathValueSymbol m5 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m5", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m5);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m5.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.SkewHerm));
        assertTrue(properties.contains(MatrixProperties.Invertible));
    }

    /**
     * checks if m6 is hermitian (not possible until complex matrix support)
     */
    @Ignore @Test
    public void hermitianTest(){
        //C^{3,3} m6 = [2 1-i 4+2i;1+i 1 2-i;4-2i 2+i -2];
        final MathValueSymbol m6 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m6", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m6);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m6.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(properties.contains(MatrixProperties.Square));
        assertTrue(properties.contains(MatrixProperties.Norm));
        assertTrue(properties.contains(MatrixProperties.Diag));
        assertTrue(properties.contains(MatrixProperties.Herm));

    }

    /**
     * checks if m7 is not square
     */
    @Test
    public void notSquareTest(){
        //Q^{2,3} m7 = [2 0 ;0 0 ;0 2];
        final MathValueSymbol m7 = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabMatrixProperties.m7", MathValueSymbol.KIND).orElse(null);
        assertNotNull(m7);
        MathMatrixArithmeticValueSymbol symbol = ((MathMatrixArithmeticValueSymbol)m7.getValue());
        ArrayList<MatrixProperties> properties = symbol.getMatrixProperties();
        assertTrue(m7.getValue() instanceof MathMatrixArithmeticValueSymbol);
        assertFalse(properties.contains(MatrixProperties.Square));
    }

    @Test
    public void testStaticProperty() throws IOException {
        MathParser parser = new MathParser();
        ASTMathAssignmentDeclarationStatement ast = parser.parse_StringMathAssignmentDeclarationStatement("static Q someValue = 1;").orElse(null);
        assertNotNull(ast);
    }

    @Test
    /**
     * We are happy if not assertion or exception is triggered
     * Matrix values can not be resolved.
     * see https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/languages/MontiMath/issues/3
     */
    public void createMatrixBySubmatricesTest() {
        final MathValueSymbol b = symTab.<MathValueSymbol>resolve("matrix.CreateMatrixBySubmatrices.B", MathValueSymbol.KIND).orElse(null);
        assertNotNull(b);
    }


}
