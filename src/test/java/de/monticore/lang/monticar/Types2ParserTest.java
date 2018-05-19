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
package de.monticore.lang.monticar;

import static org.junit.Assert.assertNotNull;

import java.io.IOException;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.lang.monticar.types2._ast.ASTElementType;
import de.monticore.lang.monticar.types2._parser.Types2Parser;
import org.junit.Test;

/**
 * Created by MichaelvonWenckstern on 20.12.2017.
 */
public class Types2ParserTest {
  @Test
  public void testDegreeElementType() throws IOException {
    Types2Parser parser = new Types2Parser();
    ASTElementType ast = parser.parse_StringElementType("Q(-90°:90°)").orElse(null);
    assertNotNull(ast);
  }
}
