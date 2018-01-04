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

import de.monticore.ast.ASTNode;
import de.monticore.lang.math.math._ast.*;
import de.monticore.lang.math.math._parser.MathParser;
import de.monticore.lang.math.math._visitor.MathVisitor;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.se_rwth.commons.Joiners;
import org.jscience.mathematics.number.Rational;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.numberunit._ast.*;

/**
 * Created by michaelvonwenckstern on 11.02.17.
 */
public class ASTTest {
    @Test
    public void testElementType() throws IOException {
        MathParser parser = new MathParser();
        ASTElementType ast = parser.parseString_ElementType("Q").orElse(null);
        assertNotNull(ast);

        ast = parser.parseString_ElementType("Q(1 : 3 : 7)").orElse(null);
        assertNotNull(ast);

        ast = parser.parseString_ElementType("Q(10 km : 20 km : 30 km)").orElse(null);
        assertNotNull(ast);

        ast = parser.parseString_ElementType("Q(10 km : 20 km : 30 km)^{10, 15}").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testAssignment() throws IOException {
        MathParser parser = new MathParser();
        ASTMathAssignmentDeclarationExpression ast = parser.parseString_MathAssignmentDeclarationExpression("Q^{2,2} A = [1 2; 3 4];").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testArithmeticExpression() throws IOException {
        MathParser parser = new MathParser();
        ASTMathArithmeticExpression ast = parser.parseString_MathArithmeticExpression("1 m/s + 7 m/s").orElse(null);
        assertNotNull(ast);

        final List<ASTUnitNumber> numbers = new ArrayList<>();
        MathVisitor visitor = new MathVisitor() {
            @Override
            public void visit(ASTUnitNumber node) {
                numbers.add(node);
            }
        };
        ast.accept(visitor);

        assertEquals(2, numbers.size());

        List<Integer> iList = numbers.stream().map(n -> n.getNumber().get().intValue())
                .collect(Collectors.toList());
        assertTrue(iList.containsAll(Arrays.asList(1, 7)));
    }

    @Test
    public void testArithmeticExpression2() throws IOException {
        MathParser parser = new MathParser();
        ASTMathArithmeticExpression ast = parser.parseString_MathArithmeticExpression("1/2 + 3/2").orElse(null);
        assertNotNull(ast);

        final List<ASTUnitNumber> numbers = new ArrayList<>();
        MathVisitor visitor = new MathVisitor() {
            @Override
            public void visit(ASTUnitNumber node) {
                numbers.add(node);
            }
        };
        ast.accept(visitor);

        assertEquals(2, numbers.size());

        List<Rational> iList = numbers.stream().map(n -> n.getNumber().get())
                .collect(Collectors.toList());
        assertTrue(iList.containsAll(Arrays.asList(Rational.valueOf(1, 2), Rational.valueOf(3, 2))));
    }

    @Test
    public void testArithmeticExpression3() throws IOException {
        MathParser parser = new MathParser();
        ASTMathArithmeticExpression ast = parser.parseString_MathArithmeticExpression("2 - 3i * -4 + 3i").orElse(null);
        assertNotNull(ast);

        final List<ASTComplexNumber> numbers = new ArrayList<>();
        final List<Boolean> mul = new ArrayList<>();
        MathVisitor visitor = new MathVisitor() {
            @Override
            public void visit(ASTComplexNumber node) {
                numbers.add(node);
            }

            public void visit(ASTMathArithmeticMultiplicationExpression node) {
                mul.add(true);
            }

            public void visit(ASTMathArithmeticMatrixMultiplicationExpression node) {
                mul.add(true);
            }

        };
        ast.accept(visitor);

        assertEquals(2, numbers.size());
        assertEquals(1, mul.size()); // so that we are sure to have exactly one infix * AST

        assertEquals(Rational.valueOf(2, 1), numbers.get(0).getReal());
        assertEquals(Rational.valueOf(-3, 1), numbers.get(0).getImg());

        assertEquals(Rational.valueOf(-4, 1), numbers.get(1).getReal());
        assertEquals(Rational.valueOf(3, 1), numbers.get(1).getImg());
    }

    @Test
    public void testArithmeticExpression4() throws IOException {
        MathParser parser = new MathParser();
        ASTMathAssignmentExpression ast = parser.parseString_MathAssignmentExpression("v = (1+m/s);").orElse(null);
        assertNotNull(ast);

        final List<ASTNode> numbers = new ArrayList<>();
        final List<Boolean> div = new ArrayList<>();
        final List<String> names = new ArrayList<>();
        MathVisitor visitor = new MathVisitor() {
            @Override
            public void visit(ASTComplexNumber node) {
                numbers.add(node);
            }

            @Override
            public void visit(ASTUnitNumber node) {
                numbers.add(node);
            }

            public void visit(ASTMathArithmeticDivisionExpression node) {
                div.add(true);
            }

            public void visit(ASTMathNameExpression node) {

                names.add(node.getName());

            }

            public void visit(ASTMathMatrixNameExpression node) {
                names.add(node.getName().get());
            }

            public void visit(ASTMathAssignmentExpression node) {
                if (node.getName().isPresent())
                    names.add(node.getName().get());
                else if(node.getDottedName().isPresent())
                    names.add(Joiners.DOT.join(node.getDottedName().get().getNames()));
                else if(node.getMathMatrixNameExpression().isPresent())
                    names.add(node.getMathMatrixNameExpression().get().getName().get());//TOdo add access info?
            }

        };
        ast.accept(visitor);

        assertEquals(1, numbers.size()); //the 1
        assertEquals(3, names.size());
        assertEquals(1, div.size()); // so that we are sure to have exactly one infix / AST

        assertTrue(names.containsAll(Arrays.asList("v", "m", "s")));
    }

    @Test
    public void testMatrix() throws IOException {
        MathParser parser = new MathParser();
        ASTMathMatrixValueExplicitExpression ast = parser.parseString_MathMatrixValueExplicitExpression("[1 2; 3 4]").orElse(null);
        assertNotNull(ast);

    }

    @Test
    public void testAdd1() throws IOException {
        MathParser parser = new MathParser();
        ASTMathCompilationUnit ast = parser.parse("src/test/resources/Calculations/add1.m").orElse(null);
        assertNotNull(ast);
    }
}
