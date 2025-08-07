/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import static de.monticore.lang.monticar.emadl.ParserTest.ENABLE_FAIL_QUICK;

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


    @Ignore
    @Test
    public void testModularInstances(){

        Scope symTab = createSymTab("src/test/resources/models");

        EMAComponentInstanceSymbol mainInstance = symTab.<EMAComponentInstanceSymbol>resolve("ModularInstanceTest.mainB", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net1", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net2 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net2", EMAComponentInstanceSymbol.KIND).get();

        assertEquals(null, "ModularInstanceTest.mainB.net1", net1.getFullName());
        assertEquals(null, "ModularInstanceTest.mainB.net2", net2.getFullName());

        EMAComponentInstanceSymbol net1C = net1.getSpannedScope().<EMAComponentInstanceSymbol>resolve("netC", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net1D = net1.getSpannedScope().<EMAComponentInstanceSymbol>resolve("netD", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net2C = net2.getSpannedScope().<EMAComponentInstanceSymbol>resolve("netC", EMAComponentInstanceSymbol.KIND).get();
        EMAComponentInstanceSymbol net2D = net2.getSpannedScope().<EMAComponentInstanceSymbol>resolve("netD", EMAComponentInstanceSymbol.KIND).get();

        assertEquals(null, "ModularInstanceTest.mainB.net1.netC", net1C.getFullName());
        assertEquals(null, "ModularInstanceTest.mainB.net1.netD", net1D.getFullName());
        assertEquals(null, "ModularInstanceTest.mainB.net2.netC", net2C.getFullName());
        assertEquals(null, "ModularInstanceTest.mainB.net2.netD", net2D.getFullName());

        ArchitectureSymbol arch1C = net1C.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        ArchitectureSymbol arch1D = net1D.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

        ArchitectureSymbol arch2C = net2C.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        ArchitectureSymbol arch2D = net2D.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

        // TODO: java.lang.IllegalStateException: The following names could not be resolved: convChannels,convChannels

        arch1C.resolve();
//        arch1D.resolve();
//        arch2C.resolve();
//        arch2D.resolve();


//        int convChannels1 = ((LayerSymbol)arch1D.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
//        int convChannels2 = ((LayerSymbol)arch2D.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
//        int units1 = ((LayerSymbol)arch1D.getStreams().get(0).getElements().get(7)).getArgument("units").get().getRhs().getIntValue().get();
//        int units2 = ((LayerSymbol)arch2D.getStreams().get(0).getElements().get(7)).getArgument("units").get().getRhs().getIntValue().get();
//        assertEquals(20, convChannels1);
//        assertEquals(30, convChannels2);
//        assertEquals(units1, units2);
    }

}
