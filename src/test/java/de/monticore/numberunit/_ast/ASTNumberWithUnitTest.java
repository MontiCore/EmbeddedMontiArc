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
package de.monticore.numberunit._ast;

import de.monticore.numberunit._parser.NumberUnitParser;
import org.junit.Test;

import javax.measure.unit.Unit;
import java.io.IOException;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * @author Christoph Richter
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