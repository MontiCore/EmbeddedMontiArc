/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math;

import de.monticore.lang.math._ast.*;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.matrix._ast.ASTMathMatrixAccessExpression;
import de.monticore.lang.matrix._ast.ASTMathMatrixValueExplicitExpression;
import de.se_rwth.commons.logging.Log;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;
import java.util.Optional;

import static de.monticore.lang.math.PrintAST.printAST;
import static org.junit.Assert.*;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class ASTTest {

    // old ---------------------------------------------------------------------

    @Test
    public void testElementType() throws IOException {
        MathParser parser = new MathParser();
        ASTElementType ast = parser.parse_StringElementType("Q").orElse(null);
        assertNotNull(ast);

        ast = parser.parse_StringElementType("Q(1 : 3 : 7)").orElse(null);
        assertNotNull(ast);

        ast = parser.parse_StringElementType("Q(10 km : 20 km : 30 km)").orElse(null);
        assertNotNull(ast);

        ast = parser.parse_StringElementType("Q(10 km : 20 km : 30 km)^{10, 15}").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testAssignment() throws IOException {
        MathParser parser = new MathParser();
        ASTMathAssignmentDeclarationStatement ast = parser.parse_StringMathAssignmentDeclarationStatement("Q^{2,2} A = [1 2; 3 4];").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testMatrix() throws IOException {
        MathParser parser = new MathParser();
        ASTMathMatrixValueExplicitExpression ast = parser.parse_StringMathMatrixValueExplicitExpression("[1 2; 3 4]").orElse(null);
        assertNotNull(ast);

    }

    @Test
    public void testAdd1() throws IOException {
        MathParser parser = new MathParser();
        ASTMathCompilationUnit ast = parser.parse("src/test/resources/Calculations/add1.m").orElse(null);
        assertNotNull(ast);
    }

    // new ---------------------------------------------------------------

    @BeforeClass
    public static void init() {
        Log.enableFailQuick(false);
    }

    @Test
    public void ASTComplexTest1() throws Exception {
        MathParser parser = new MathParser();
        Optional<de.monticore.numberunit._ast.ASTComplexNumber> ast = parser.parse_StringComplexNumber("1+2i");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTComplexTest2() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathScript> ast = parser.parse_StringMathScript("script a Q^2+1i a; end");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        assertFalse((ast.get().getStatements().getStatementList().isEmpty()));
    }

    @Test
    public void ASTForTest1() throws Exception {
        MathParser parser = new MathParser();
        // Optional<ASTMathForLoopExpression> ast = parser.parse_StringMathForLoopExpression("for a = b 1 + c end"); // works
        // Optional<ASTMathForLoopExpression> ast = parser.parse_StringMathForLoopExpression("for a = b c end"); // works
        Optional<ASTMathForLoopExpression> ast = parser.parse_StringMathForLoopExpression("for a = b d = c + 1; end"); // does not work
        //      Optional<ASTMathForLoopExpression> ast = parser.parse_StringMathForLoopExpression("for a = b 1=1+hallo+3 end");
        System.out.println(printAST(ast.get()));
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTForTest2() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathForLoopExpression> ast = parser.parse_StringMathForLoopExpression("for a = b a++; for c= 1 b= a+c; end; d= 4+a; end");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTIfTest1() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathIfStatement> ast = parser.parse_StringMathIfStatement("if (a == b) b=a+1; end ");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTMatrix1Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathMatrixValueExplicitExpression> ast = parser.parse_StringMathMatrixValueExplicitExpression("[1,2,3;4,5,6] ");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTMatrix2Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathMatrixValueExplicitExpression> ast = parser.parse_StringMathMatrixValueExplicitExpression("[1 2 3;4 5 6] ");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTMatrixTest3() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathMatrixValueExplicitExpression> ast = parser.parse_StringMathMatrixValueExplicitExpression("[1+3i, 4+5i ;8 9] ");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTDeclaration1Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathDeclarationStatement> ast = parser.parse_StringMathDeclarationStatement("Q^{1,2} a");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTDeclaration2Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathDeclarationStatement> ast = parser.parse_StringMathDeclarationStatement("N(1:2:6)^1 a");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

  /*  @Test
    public void ASTDeclarationInvalidTest() throws Exception {
        //A is no ElementType
        MathParser parser = new MathParser();
        Optional<ASTMathDeclarationStatement> ast = parser.parse_StringMathDeclarationStatement("A(1:2:6)^1 a");
        assertFalse(ast.isPresent());
        assertTrue(parser.hasErrors());
    }*/

    @Test
    public void ASTAssignmentDeclaration1Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Q(1:2:6)^1 a = 1+2*4");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTAssignmentDeclaration2Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Z^{3,7} nsu1je = [1,2;3,4]");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

   /* @Test
    public void ASTAssignmentDeclarationInvalid1Test() throws Exception {
        //E is no ElementType
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("E^{3,7} nsu1je = [1,2;3,4]");
        assertFalse(ast.isPresent());
        assertTrue(parser.hasErrors());
    }*/

    @Test
    public void ASTAssignmentDeclarationInvalid2Test() throws Exception {
        //== is not allowed
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Q^{3,7} nsu1je == [1,2;3,4]");
        assertFalse(ast.isPresent());
        assertTrue(parser.hasErrors());
    }

    @Test
    public void ASTAssignmentDeclarationInvalid3Test() throws Exception {
        //name is missing
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Q^3 = [1 2 3]");
        assertFalse(ast.isPresent());
        assertTrue(parser.hasErrors());
    }

    @Test
    public void ASTAssignment1Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentStatement> ast = parser.parse_StringMathAssignmentStatement("A = [1,2,2]");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTAssignment2Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentStatement> ast = parser.parse_StringMathAssignmentStatement("A.r = 3:4:5");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTAssignment3Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("B(1:3) a = [4;6]");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        System.out.println(printAST(ast.get()));
    }

    @Test
    public void ASTNameExpressionTest1() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTNameExpression> ast = parser.parse_StringNameExpression("Z");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTAssignment4Test() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathDeclarationStatement> ast = parser.parse_StringMathDeclarationStatement("Z^1 a");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTDottedNameExpressionTest1() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathDottedNameExpression> ast = parser.parse_StringMathDottedNameExpression("name.name2");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
    }

    @Test
    public void ASTTest() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathCompilationUnit> ast = parser.parse_StringMathCompilationUnit("script S\n" +
                "Q^2 a = a+b;\n" +
                " end");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        System.out.println(printAST(ast.get()));
    }

    @Test
    public void MathMatrixValueExplicitExpressionMinusTest() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Z^{2,2} A = [-1, -1; -1, -1]");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        Log.debug(PrintAST.printAST(ast.get()), "MathMatrixValueExplicitExpression");
        ASTMathMatrixValueExplicitExpression matrix = ((ASTMathMatrixValueExplicitExpression) ast.get().getExpression());
        // assert that result has dimentsions {2,2}
        assertEquals(2, matrix.getMathMatrixAccessExpressionList().size());
        for (ASTMathMatrixAccessExpression accessExpr : matrix.getMathMatrixAccessExpressionList())
            assertEquals(2, accessExpr.getMathMatrixAccessList().size());
    }

    @Test
    public void MathMatrixValueExplicitExpressionTest() throws Exception {
        MathParser parser = new MathParser();
        Optional<ASTMathAssignmentDeclarationStatement> ast = parser.parse_StringMathAssignmentDeclarationStatement("Z^{2,2} A = [1, 1; 1, 1]");
        assertTrue(ast.isPresent());
        assertFalse(parser.hasErrors());
        Log.debug(PrintAST.printAST(ast.get()), "MathMatrixValueExplicitExpression");
        ASTMathMatrixValueExplicitExpression matrix = ((ASTMathMatrixValueExplicitExpression) ast.get().getExpression());
        // assert that result has dimentsions {2,2}
        assertEquals(2, matrix.getMathMatrixAccessExpressionList().size());
        for (ASTMathMatrixAccessExpression accessExpr : matrix.getMathMatrixAccessExpressionList())
            assertEquals(2, accessExpr.getMathMatrixAccessList().size());
    }

    @Test
    public void minusAssignmentTest() throws Exception {
        MathParser parser = new MathParser();
        ASTMathAssignmentStatement ast = parser.parse_StringMathAssignmentStatement("x = -y").orElse(null);
        assertNotNull(ast);
    }
}
