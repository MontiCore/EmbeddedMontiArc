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
package de.monticore.lang.math.math;

import org.jscience.mathematics.number.Rational;
import org.jscience.mathematics.vector.DenseMatrix;
import org.junit.Test;

import javax.measure.unit.Unit;
import java.io.IOException;

import static org.junit.Assert.assertTrue;


/**
 * Created by Tobias PC on 25.01.2017.
 */
public class GeneratorTest {
    /*
    @Test
    public void testInitialization() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "Initialization");
        assertTrue(g.compileGeneratedClass("Initialization"));

    }

    @Test
    public void testInitialization_execute() throws Exception {
        testInitialization();
        Executable in = (Executable) Generator.getInstanceOfGeneratedClass("Initialization");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + Rational.valueOf(1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + Rational.valueOf(2, 1) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(1, 1)}
                , {Rational.valueOf(1, 1), Rational.valueOf(1, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("D : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(1, 2)}
                , {Rational.valueOf(1, 3), Rational.valueOf(1, 4)}}) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        in.execute();
    }

    @Test
    public void testAssignments() throws Exception {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "Assignments");
        assertTrue(g.compileGeneratedClass("Assignments"));
    }

    @Test
    public void testAssignments_execute() throws Exception {
        testAssignments();
        Executable a = Generator.getInstanceOfGeneratedClass("Assignments");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + Rational.valueOf(2, 1) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(1, 2)}
                , {Rational.valueOf(1, 3), Rational.valueOf(1, 4)}}) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(1, 2)}
                , {Rational.valueOf(1, 3), Rational.valueOf(1, 4)}}) + " Unit : " + Unit.ONE);
        // System.out.println("D : " + Rational.valueOf(2, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        a.execute();
    }

    @Test
    public void testMathExpressions() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "MathExpressions");
        assertTrue(g.compileGeneratedClass("MathExpressions"));
    }

    @Test
    public void testMathExpressions_execute() throws Exception {
        testMathExpressions();
        Executable m = Generator.getInstanceOfGeneratedClass("MathExpressions");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + Rational.valueOf(3, 1) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + Rational.valueOf(6, 1) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + Rational.valueOf(441, 1) + " Unit : " + Unit.ONE);
        // System.out.println("D : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(42, 1), Rational.valueOf(42, 1)}
                , {Rational.valueOf(42, 1), Rational.valueOf(42, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("E : " + Rational.valueOf(4, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        m.execute();
    }

    @Test
    public void testUnits() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "Units");
        assertTrue(g.compileGeneratedClass("Units"));
    }

    @Test
    public void testUnits_execute() throws Exception {
        testUnits();
        Executable u = Generator.getInstanceOfGeneratedClass("Units");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + Rational.valueOf(5, 1) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("B : " + Rational.valueOf(1, 1) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("C : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(2, 1)}}) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("F : " + Rational.valueOf(2, 1) + " Unit : " + Unit.valueOf("m*s"));
        // System.out.println("D : " + Rational.valueOf(1, 1) + " Unit : " + Unit.valueOf("m^2"));
        // System.out.println("E : " + Rational.valueOf(441, 1) + " Unit : " + Unit.valueOf("m^4"));
        // System.out.println("Actual Output:");
        u.execute();
    }

    @Test
    public void testUnits2() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "Units2");
        assertTrue(g.compileGeneratedClass("Units2"));
    }

    @Test
    public void testUnits2_execute() throws Exception {
        testUnits2();
        Executable u = Generator.getInstanceOfGeneratedClass("Units2");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(2, 1)}}) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("B : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1)}, {Rational.valueOf(2, 1)}}) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("C : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(5, 1)}}) + " Unit : " + Unit.valueOf("m^2"));
        // System.out.println("D : " + Rational.valueOf(2, 1) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("E : " + Rational.valueOf(5, 1) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("F : " + Rational.valueOf(13, 1) + " Unit : " + Unit.valueOf("m^2"));
        // System.out.println("Actual Output:");
        u.execute();
    }

    @Test
    public void testMatrixWithMathExpressions() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "MatrixWithMathExpression");
        assertTrue(g.compileGeneratedClass("MatrixWithMathExpression"));
    }

    @Test
    public void testMatrixWithMathExpressions_execute() throws Exception {
        testMatrixWithMathExpressions();
        Executable matrixWithMathExpression = Generator.getInstanceOfGeneratedClass("MatrixWithMathExpression");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("B : " + Rational.valueOf(5, 1) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + Rational.valueOf(3, 1) + " Unit : " + Unit.ONE);
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(2, 1), Rational.valueOf(160, 25)},
                {Rational.valueOf(-160, 25), Rational.valueOf(-94, 25)}}) + " Unit : " + Unit.ONE);
        // System.out.println("D : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(2, 1), Rational.valueOf(8, 1)},
                {Rational.valueOf(4, 1), Rational.valueOf(10, 1)},
                {Rational.valueOf(6, 1), Rational.valueOf(12, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        matrixWithMathExpression.execute();
    }

    @Test
    public void testPowerWise() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "PowerWise");
        assertTrue(g.compileGeneratedClass("PowerWise"));
    }

    @Test
    public void testPowerWise_execute() throws Exception {
        testPowerWise();
        Executable pw = Generator.getInstanceOfGeneratedClass("PowerWise");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(4, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(49, 1), Rational.valueOf(100, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        pw.execute();
    }

    @Test
    public void testTimeWise() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "TimeWise");
        assertTrue(g.compileGeneratedClass("TimeWise"));
    }

    @Test
    public void testTimeWise_execute() throws Exception {
        testTimeWise();
        Executable tw = Generator.getInstanceOfGeneratedClass("TimeWise");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(2, 1), Rational.valueOf(4, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(20, 1), Rational.valueOf(36, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        tw.execute();
    }

    @Test
    public void testSolEqu() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "SolEqu");
        assertTrue(g.compileGeneratedClass("SolEqu"));
    }

    @Test
    public void testSolEqu_execute() throws Exception {
        testSolEqu();
        Executable se = Generator.getInstanceOfGeneratedClass("SolEqu");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(3, 11)}, {Rational.valueOf(1, 11)}, {Rational.valueOf(7, 22)}}) + " Unit : " + Unit.ONE);
        // System.out.println("B : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(3, 11)}, {Rational.valueOf(1, 11)}, {Rational.valueOf(7, 22)}}) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(3, 11)}, {Rational.valueOf(1, 11)}, {Rational.valueOf(7, 22)}}) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("Actual Output:");
        se.execute();
    }

    @Test
    public void testForLoop() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "ForLoop");
        assertTrue(g.compileGeneratedClass("ForLoop"));
    }

    @Test
    public void testForLoop_execute() throws Exception {
        testForLoop();
        Executable fl = Generator.getInstanceOfGeneratedClass("ForLoop");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("sum : " + Rational.valueOf(3840, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        fl.execute();
    }

    @Test
    public void testForLoop2() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "ForLoop2");
        assertTrue(g.compileGeneratedClass("ForLoop2"));
    }

    @Test
    public void testForLoop2_execute() throws Exception {
        testForLoop2();
        Executable fl2 = Generator.getInstanceOfGeneratedClass("ForLoop2");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("c : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(3, 1),
                Rational.valueOf(5, 1), Rational.valueOf(7, 1), Rational.valueOf(9, 1)}}) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("x : " + Rational.valueOf(5, 1) + " Unit : " + Unit.ONE);
        // System.out.println("y : " + Rational.valueOf(625, 1) + " Unit : " + Unit.valueOf("m^2"));
        // System.out.println("z : " + Rational.valueOf(125, 1) + " Unit : " + Unit.valueOf("m"));
        // System.out.println("Actual Output:");
        fl2.execute();
    }

    @Test
    public void testForLoop3() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "ForLoop3");
        assertTrue(g.compileGeneratedClass("ForLoop3"));
    }

    @Test
    public void testForLoop3_execute() throws Exception {
        testForLoop3();
        Executable fl3 = Generator.getInstanceOfGeneratedClass("ForLoop3");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("A : " + DenseMatrix.valueOf(new Rational[][]{{Rational.valueOf(1, 1), Rational.valueOf(2, 1), Rational.valueOf(3, 1)},
                {Rational.valueOf(4, 1), Rational.valueOf(5, 1), Rational.valueOf(6, 1)},
                {Rational.valueOf(7, 1), Rational.valueOf(8, 1), Rational.valueOf(9, 1)}}) + " Unit : " + Unit.ONE);
        // System.out.println("C : " + Rational.valueOf(45, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        fl3.execute();
    }

    @Test
    public void testIf() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "If");
        assertTrue(g.compileGeneratedClass("If"));
    }

    @Test
    public void testIf_execute() throws Exception {
        testIf();
        Executable if1 = Generator.getInstanceOfGeneratedClass("If");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("cond : " + Rational.valueOf(2, 1) + " Unit : " + Unit.ONE);
        // System.out.println("result : " + Rational.valueOf(0, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        if1.execute();
    }

    @Test
    public void testIf2() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "If2");
        assertTrue(g.compileGeneratedClass("If2"));
    }

    @Test
    public void testIf2_execute() throws Exception {
        testIf2();
        Executable if2 = Generator.getInstanceOfGeneratedClass("If2");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("cond : " + Rational.valueOf(1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("result : " + Rational.valueOf(1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("Actual Output:");
        if2.execute();
    }

    @Test
    public void testIf3() throws IOException {
        String model = "src/test/resources/Generation/";
        Generator g = new Generator();
        g.generate(model, "If3");
        assertTrue(g.compileGeneratedClass("If3"));
    }

    @Test
    public void testIf3_execute() throws Exception {
        testIf3();
        Executable if3 = Generator.getInstanceOfGeneratedClass("If3");
        //expected output
        // System.out.println("Expected Output:");
        // System.out.println("cond1 : " + Rational.valueOf(1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("cond2 : " + Rational.valueOf(-1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("result : " + Rational.valueOf(-1, 1) + " Unit : " + Unit.ONE);
        // System.out.println("bool : " + false);
        // System.out.println("Actual Output:");
        if3.execute();
    }*/


}
