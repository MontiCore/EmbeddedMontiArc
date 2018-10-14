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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import de.monticore.symboltable.Scope;


public class SimulatorTest extends AbstractSymtabTest{

    @Test
    public void resolveTest() {
        Scope symtab = createSymTab("src/test/resources/emam");
        EMAComponentInstanceSymbol model = symtab.
                <EMAComponentInstanceSymbol>resolve("simulator.mainController",
                        EMAComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(model);
    }
}