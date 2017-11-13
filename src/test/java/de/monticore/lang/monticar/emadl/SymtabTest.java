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
package de.monticore.lang.monticar.emadl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CNNArchCompilationUnitSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.CNNTrainCompilationUnitSymbol;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.monticore.symboltable.Scope;
import org.junit.Ignore;
import org.junit.Test;

public class SymtabTest extends AbstractSymtabTest {

    @Test
    public void testParsing() throws Exception {
        EMADLParser parser = new EMADLParser();
        assertTrue(parser.parse("src/test/resources/SimpleComponent.emadl").isPresent());
    }

    @Test
    public void testAlexnet(){
        Scope symTab = createSymTab("src/test/resources");
        ComponentSymbol a = symTab.<ComponentSymbol>resolve("Alexnet", ComponentSymbol.KIND).orElse(null);
        assertNotNull(a);
        CNNArchCompilationUnitSymbol archSymbol = (CNNArchCompilationUnitSymbol) a.getSpannedScope().resolve("Alexnet", CNNArchCompilationUnitSymbol.KIND).get();
        assertNotNull(a);
        CNNTrainCompilationUnitSymbol configSymbol = (CNNTrainCompilationUnitSymbol) a.getSpannedScope().resolve("SimpleConfig", CNNTrainCompilationUnitSymbol.KIND).get();
        assertNotNull(a);
    }

}
