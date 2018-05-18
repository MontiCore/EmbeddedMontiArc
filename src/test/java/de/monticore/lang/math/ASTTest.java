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

import de.monticore.ast.ASTNode;
import de.monticore.commonexpressions._ast.ASTPlusExpression;
import de.monticore.lang.math._ast.*;
import de.monticore.lang.math._parser.MathParser;
import de.monticore.lang.math._visitor.MathVisitor;
import de.monticore.lang.matrix._ast.ASTMathMatrixValueExplicitExpression;
import de.monticore.lang.monticar.commonexpressions._ast.ASTCommonAdditionExpression;
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
        ASTMathAssignmentDeclarationExpression ast = parser.parse_StringMathAssignmentDeclarationExpression("Q^{2,2} A = [1 2; 3 4];").orElse(null);
        assertNotNull(ast);
    }

    @Test
    public void testArithmeticExpression4() throws IOException {
        MathParser parser = new MathParser();
        ASTMathAssignmentExpression ast = parser.parse_StringMathAssignmentExpression("v = (1+m/s);").orElse(null);
        assertNotNull(ast);

        final List<ASTNode> numbers = new ArrayList<>();
        final List<Boolean> div = new ArrayList<>();
        final List<String> names = new ArrayList<>();
        MathVisitor visitor = new MathVisitor() {

            public void visit(ASTComplexNumber node) {
                numbers.add(node);
            }

            public void visit(ASTUnitNumber node) {
                numbers.add(node);
            }

            public void visit(ASTMathMatrixNameExpression node) {
                names.add(node.getName());
            }

            public void visit(ASTMathAssignmentExpression node) {
                if (node.getNameOpt().isPresent())
                    names.add(node.getNameOpt().get());
                else if(node.getMathDottedNameExpressionOpt().isPresent())
                    names.add(Joiners.DOT.join(node.getMathDottedNameExpressionOpt().get().getNameList()));
                else if(node.getMathMatrixNameExpressionOpt().isPresent())
                    names.add(node.getMathMatrixNameExpressionOpt().get().getName());//TOdo add access info?
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
        ASTMathMatrixValueExplicitExpression ast = parser.parse_StringMathMatrixValueExplicitExpression("[1 2; 3 4]").orElse(null);
        assertNotNull(ast);

    }

    @Test
    public void testAdd1() throws IOException {
        MathParser parser = new MathParser();
        ASTMathCompilationUnit ast = parser.parse("src/test/resources/Calculations/add1.m").orElse(null);
        assertNotNull(ast);
    }
}
