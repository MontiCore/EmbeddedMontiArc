/**
 * ******************************************************************************
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
package de.monticore.lang.embeddedmontiview;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.IOError;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTRanges;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._parser.EmbeddedMontiViewParser;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ComponentScope;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.PortSymbol;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.si._symboltable.SIUnitRangesSymbolReference;
import org.junit.Assert;
import org.junit.Test;

public class SymbolPrinterTest {

  @Test
  public void PortPrintSIUnitTest() throws IOException {
    EmbeddedMontiViewParser parser = new EmbeddedMontiViewParser();
    ASTRange astRange = parser.parseString_Range("(-100 m/s^2:100 m/s^200)").get();
    SIUnitRangesSymbolReference siref = new SIUnitRangesSymbolReference("SIUnitRangesType", Arrays.asList(astRange));
    PortSymbol portSymbol = PortSymbol.builder().setDirection(true).setName("port1").setTypeReference(Optional.of(siref)).build();
    portSymbol.setEnclosingScope(new ComponentScope());

    String result = portSymbol.toString();

    //    assertEquals()
    assertFalse(result.contains("SIUnitRangesType"));
    assertTrue(result.replace(" ", "").contains("(-100/1m/s²:100/1m/s²)"));
    System.out.println(result);
  }
}
