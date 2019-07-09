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

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.CompositeElementSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;
import static junit.framework.TestCase.assertEquals;

public class InstanceTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testInstances(){
        Scope symtab = createSymTab("src/test/resources/models/");
        EMAComponentInstanceSymbol mainInstance = symtab.<EMAComponentInstanceSymbol>
                resolve("InstanceTest.mainB", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("net1", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net2 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("net2", EMAComponentInstanceSymbol.KIND).get();

        assertEquals("InstanceTest.mainB.net1",net1.getFullName());
        assertEquals("InstanceTest.mainB.net2",net2.getFullName());

        ArchitectureSymbol arch1 = net1.getSpannedScope().<ArchitectureSymbol>
                resolve("", ArchitectureSymbol.KIND).get();

        ArchitectureSymbol arch2 = net2.getSpannedScope().<ArchitectureSymbol>
                resolve("", ArchitectureSymbol.KIND).get();

        arch1.resolve();
        arch2.resolve();

        int convChannels1 = ((LayerSymbol)arch1.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
        int convChannels2 = ((LayerSymbol)arch2.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();

        assertEquals(20, convChannels1);
        assertEquals(40, convChannels2);
    }
}
