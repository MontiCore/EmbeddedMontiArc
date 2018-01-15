/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */

package de.monticore.lang.math.math;


import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.math.math._symboltable.*;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathNumberExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathValueSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixArithmeticValueSymbol;
import de.monticore.lang.math.math._symboltable.matrix.MathMatrixExpressionSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;
import org.junit.Ignore;
import org.junit.Test;

import javax.measure.unit.Unit;
import java.nio.file.Paths;

import static org.junit.Assert.*;

import de.monticore.lang.numberunit._ast.*;

public class MathSymbolTableCreatorTest {

    // this is a very simple test and only supports one use case
    // it does not even support right now
    // Q(0 : 10 km)^{3,1} j
    // it is only used to show symbol adaption in the embedded montiarc math
    // project
    @Test
    public void test0() {
        Scope symTab = createSymTab("src/test/resources");

        //Q(0 km : 10 km)^{3,1} j = [ 2 mm; 3 cm;4 km ];
        final MathValueSymbol j = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabTest0.j", MathValueSymbol.KIND).orElse(null);
        Log.info(symTab.toString(), "SCOPE:");
        assertNotNull(j);

        Log.info(j.getTextualRepresentation(), "Result:");
        /*assertEquals(2, j.getDimensions().size());
        assertEquals(3, (int) j.getDimensions().get(0));
        assertEquals(1, (int) j.getDimensions().get(1));

        assertTrue(j.getRange().getStart().isPresent());

        assertEquals(new ASTUnitNumber(Rational.ZERO, Unit.valueOf("km")),
                j.getRange().getStart().get());
        assertTrue(j.getRange().getEnd().isPresent());
        assertEquals(new ASTUnitNumber(Rational.valueOf(10, 1), Unit.valueOf("km")),
                j.getRange().getEnd().get());
    */
    }

    /**
     * test to check mathexpressions
     */
    @Test
    public void test1() {
        Scope symTab = createSymTab("src/test/resources");

        //Q a = 2+7*5+4%3;
        final MathValueSymbol mathSymbol1 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.a", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        //check if its (2) + (7*5 + 4%3)
        /*
        assertTrue(mathSymbol1.getValue() instanceof MathExpression);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue() instanceof JSValue);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue() instanceof MathExpression);
        assertEquals(((MathExpression) mathSymbol1.getValue()).getOp(), Operator.Plus);
        //check if its (7*5) + (4%3)
        assertTrue(((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(0).getValue() instanceof MathExpression);
        assertTrue(((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(1).getValue() instanceof MathExpression);
        assertEquals(((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOp(), Operator.Plus);
        //check if its (7)*(5)
        assertTrue(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(0).getValue())).getOperands().get(0).getValue() instanceof JSValue);
        assertTrue(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(0).getValue())).getOperands().get(1).getValue() instanceof JSValue);
        assertEquals(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(0).getValue())).getOp(), Operator.Times);
        //check if its (4)%(3)
        assertTrue(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(1).getValue())).getOperands().get(0).getValue() instanceof JSValue);
        assertTrue(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(1).getValue())).getOperands().get(1).getValue() instanceof JSValue);
        assertEquals(((MathExpression) (((MathExpression) ((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue()).getOperands().get(1).getValue())).getOp(), Operator.Mod);
    */
    }

    /**
     * test to check the dims, mathexpressions and values of a matrix
     */
    @Test
    public void test2() {
        Scope symTab = createSymTab("src/test/resources");

        //Q^{2,2} m1 = [23, 44; 22, 222] + [ 1, 2; 3,4] * [3,4;5,6] - [7,8;9,10].*[22,1;2,3];
        final MathValueSymbol mathSymbol1 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.m1", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        //check dimensions of the variable
        /*assertEquals(2, mathSymbol1.getDimensions().size());
        assertEquals(2, (int) mathSymbol1.getDimensions().get(0));
        assertEquals(2, (int) mathSymbol1.getDimensions().get(1));
        //check ([23, 44; 22, 222]) + ([ 1, 2; 3,4] * [3,4;5,6] - [7,8;9,10].*[22,1;2,3])
        assertTrue(mathSymbol1.getValue() instanceof MathExpression);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue() instanceof JSMatrix);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue() instanceof MathExpression);
        assertEquals(((MathExpression) mathSymbol1.getValue()).getOp(), Operator.Plus);
        //check dimensions of [23, 44; 22, 222]
        assertEquals(((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getColDimension(), 2);
        assertEquals(((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getRowDimension(), 2);
        //check values of [23, 44; 22, 222]
        assertEquals(((JSValue) ((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getElement(0, 0)).getRealNumber(), Rational.valueOf(23, 1));
        assertEquals(((JSValue) ((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getElement(0, 1)).getRealNumber(), Rational.valueOf(44, 1));
        assertEquals(((JSValue) ((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getElement(1, 0)).getRealNumber(), Rational.valueOf(22, 1));
        assertEquals(((JSValue) ((JSMatrix) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getElement(1, 1)).getRealNumber(), Rational.valueOf(222, 1));
    */
    }

    /**
     * test some declarations/definitions
     */
    @Test
    public void test3() {
        Scope symTab = createSymTab("src/test/resources");

        //Q b = -7;
        final MathValueSymbol mathSymbol1 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.b", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        /*assertTrue(mathSymbol1.getValue() instanceof JSValue);
        assertEquals(((JSValue) mathSymbol1.getValue()).getRealNumber(), Rational.valueOf(-7, 1));
*/
        //Q c = 9;
        final MathValueSymbol mathSymbol2 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.c", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol2);
        /*assertTrue(mathSymbol2.getValue() instanceof JSValue);
        assertEquals(((JSValue) mathSymbol2.getValue()).getRealNumber(), Rational.valueOf(9, 1));
*/
        //Q d = b + c;
        final MathValueSymbol mathSymbol3 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.d", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol3);
        // checks (b) + (c)
        /*assertTrue(mathSymbol3.getValue() instanceof MathExpression);
        assertTrue(((MathExpression) mathSymbol3.getValue()).getOperands().get(0).getValue() instanceof MathValueReference);
        assertTrue(((MathExpression) mathSymbol3.getValue()).getOperands().get(1).getValue() instanceof MathValueReference);
        assertEquals(((MathExpression) mathSymbol3.getValue()).getOp(), Operator.Plus);
        // checks the names of the references
        assertEquals(((MathValueReference) ((MathExpression) mathSymbol3.getValue()).getOperands().get(0).getValue()).getName(), "b");
        assertEquals(((MathValueReference) ((MathExpression) mathSymbol3.getValue()).getOperands().get(1).getValue()).getName(), "c");
*/
    }

    /**
     * test with rational arithmetic math expressions with units
     */
    @Test
    public void test4() {
        Scope symTab = createSymTab("src/test/resources");

        //Rational h = 2 km + 7 m * 3 m ^ 3;
        final MathValueSymbol mathSymbol1 = symTab.<MathValueSymbol>resolve("symtab.SymtabTest.h", MathValueSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol1);
        //checks (2 km) + (7 m * 3 m ^ 3)
        /*assertTrue(mathSymbol1.getValue() instanceof MathExpression);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue() instanceof JSValue);
        assertTrue(((MathExpression) mathSymbol1.getValue()).getOperands().get(1).getValue() instanceof MathExpression);
        assertEquals(((MathExpression) mathSymbol1.getValue()).getOp(), Operator.Plus);
        // checks the value and unit for the virst expression
        assertEquals(((JSValue) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getRealNumber(), Rational.valueOf(2000, 1));
        assertEquals(((JSValue) ((MathExpression) mathSymbol1.getValue()).getOperands().get(0).getValue()).getUnit(), Unit.valueOf("m"));
    */
    }

    /**
     * checks the symtab for a complex matrix
     */
    @Test
    public void testComplexMatrix() {
        Scope symTab = createSymTab("src/test/resources");

        //C(0:10)^{2,2} matrix = [3+5i -6-5i; -3+2i 3-8i];
        final MathValueSymbol matrix = symTab.<MathValueSymbol>resolve
                ("matrix.ComplexNumber.matrix", MathValueSymbol.KIND).orElse(null);
        assertNotNull(matrix);
        //check the matrix dimensions
        assertEquals(2, matrix.getType().getDimensions().size());
        assertEquals("2", matrix.getType().getDimensions().get(0).getTextualRepresentation());
        assertEquals("2", matrix.getType().getDimensions().get(1).getTextualRepresentation());
        //checks the value matrix(0,0) = 3+5i
        assertTrue(matrix.getValue().isMatrixExpression());
        MathMatrixExpressionSymbol matrixExpressionSymbol = (MathMatrixExpressionSymbol) matrix.getValue();
        assertTrue(matrixExpressionSymbol.isValueExpression());
        MathMatrixArithmeticValueSymbol matrixValueExp = (MathMatrixArithmeticValueSymbol) matrixExpressionSymbol;
        assertEquals(2, matrixValueExp.getVectors().size());
        assertEquals(2, matrixValueExp.getVectors().get(0).getMathMatrixAccessSymbols().size());
        assertEquals(2, matrixValueExp.getVectors().get(1).getMathMatrixAccessSymbols().size());
        MathExpressionSymbol expressionSymbol = matrixValueExp.getVectors().get(1).getMathMatrixAccessSymbols().get(0).getMathExpressionSymbol().get();
        assertTrue(expressionSymbol.isValueExpression());
        MathNumberExpressionSymbol value = (MathNumberExpressionSymbol) expressionSymbol;
        assertTrue(value.getValue().isComplex());
        // assertTrue(( matrix.getValue()).getElement(0, 0) instanceof JSValue);
        /*assertEquals(((JSValue) ((JSMatrix) matrix.getValue()).getElement(0, 0)).getRealNumber(), Rational.valueOf(3, 1));
        assertEquals(((JSValue) ((JSMatrix) matrix.getValue()).getElement(0, 0)).getImagNumber().get(), Rational.valueOf(5, 1));
*/
    }

    /**
     * checks for boolean type
     */
    @Test
    public void test6() {
        Scope symTab = createSymTab("src/test/resources");
        //B bool = j == i';
        final MathValueSymbol matrix = symTab.<MathValueSymbol>resolve
                ("symtab.SymtabTest.bool", MathValueSymbol.KIND).orElse(null);
        assertNotNull(matrix);
        //checks (j) == (i)
        /*assertTrue(matrix.getValue() instanceof LogicalExpression);
        assertEquals(((LogicalExpression) matrix.getValue()).getOp(), Operator.Equals);
        assertTrue(((LogicalExpression) matrix.getValue()).getOperands().get(0).getValue() instanceof MathValueReference);
        assertTrue(((LogicalExpression) matrix.getValue()).getOperands().get(1).getValue() instanceof MathExpression);
        //checks i'
        assertEquals(((MathExpression) ((LogicalExpression) matrix.getValue()).getOperands().get(1).getValue()).getOp(), Operator.Trans);
    */
    }

    @Test
    public void testComplexAssignmentTest() {
        Scope symTab = createSymTab("src/test/resources");
        final MathValueSymbol complexNumber = symTab.<MathValueSymbol>resolve
                ("symtab.ComplexAssignmentTest.A", MathValueSymbol.KIND).orElse(null);
        assertNotNull(complexNumber);
        assertTrue(complexNumber.getType().isComplexType());
        assertEquals(true, complexNumber.getValue() instanceof MathNumberExpressionSymbol);
        MathNumberExpressionSymbol numberExpressionSymbol = (MathNumberExpressionSymbol) complexNumber.getValue();
        testJSValue(numberExpressionSymbol.getValue(), "3/1", "4/1");
    }

    public static void testJSValue(JSValue jsValue, String realPart, String complexPart) {
        testJSValue(jsValue, realPart);
        assertTrue(jsValue.getImagNumber().isPresent());
        assertEquals(complexPart, jsValue.getImagNumber().get().toString());
    }

    public static void testJSValue(JSValue jsValue, String realPart) {
        assertTrue(jsValue.isComplex());
        assertEquals(realPart, jsValue.getRealNumber().toString());
    }

    protected static Scope createSymTab(String... modelPath) {
        ModelingLanguageFamily fam = new ModelingLanguageFamily();
        fam.addModelingLanguage(new MathLanguage());
        final ModelPath mp = new ModelPath();

        for (String m : modelPath) {
            mp.addEntry(Paths.get(m));
        }

        GlobalScope scope = new GlobalScope(mp, fam);
        return scope;
    }
}
