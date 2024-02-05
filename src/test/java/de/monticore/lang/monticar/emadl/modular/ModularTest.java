/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.emadl.modular;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.LayerSymbol;
import de.monticore.lang.monticar.emadl.AbstractSymtabTest;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;


public class ModularTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testModularSimple() {
        Scope symtab = createSymTab("src/test/resources/models/ModularMNIST");
        EMAComponentInstanceSymbol compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("modularNetworkSimple.connector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(compInstanceSymbol);

        for (int i = 1; i <= 6; i++) {
            EMAComponentInstanceSymbol predictor = compInstanceSymbol.getSpannedScope().<EMAComponentInstanceSymbol>resolve("predictor" + i, EMAComponentInstanceSymbol.KIND).get();
            assertNotNull(predictor);
        }
        EMAComponentInstanceSymbol calculator = compInstanceSymbol.getSpannedScope().<EMAComponentInstanceSymbol>resolve("cal", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(calculator);

        EMAComponentInstanceSymbol networkInstance = compInstanceSymbol.getSpannedScope().<EMAComponentInstanceSymbol>resolve("predictor1", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(networkInstance);

        EMAComponentInstanceSymbol net1Instance = networkInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net1", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net1Instance);
        assertEquals("modularNetworkSimple.connector.predictor1.net1", net1Instance.getFullName());

        EMAComponentInstanceSymbol net2Instance = networkInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net2", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net2Instance);
        assertEquals( "modularNetworkSimple.connector.predictor1.net2", net2Instance.getFullName());

        ArchitectureSymbol net1ArchSym = net1Instance.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        assertNotNull(net1ArchSym);

        ArchitectureSymbol net2ArchSym = net2Instance.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        assertNotNull(net2ArchSym);

        net1ArchSym.resolve();
        net2ArchSym.resolve();

        int channels = ((LayerSymbol)net1ArchSym.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
        int units = ((LayerSymbol)net1ArchSym.getStreams().get(0).getElements().get(3)).getArgument("units").get().getRhs().getIntValue().get();

        assertEquals(20, channels);
        assertEquals(500, units);

        Log.info("END", "MODULAR_TEST");
    }

    @Test
    public void testModularComplex() {
        Scope symtab = createSymTab("src/test/resources/models/ModularMNIST");
        EMAComponentInstanceSymbol compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("modularNetworkComplex.connector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(compInstanceSymbol);

        EMAComponentInstanceSymbol networkInstance = compInstanceSymbol.getSpannedScope().<EMAComponentInstanceSymbol>resolve("predictor1", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(networkInstance);

        EMAComponentInstanceSymbol net1Instance = networkInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net1", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net1Instance);
        assertEquals("modularNetworkComplex.connector.predictor1.net1", net1Instance.getFullName());

        ArchitectureSymbol net1ArchSym = net1Instance.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        assertNotNull(net1ArchSym);

        net1ArchSym.resolve();

        int channels = ((LayerSymbol)net1ArchSym.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
        int units = ((LayerSymbol)net1ArchSym.getStreams().get(0).getElements().get(3)).getArgument("units").get().getRhs().getIntValue().get();

        assertEquals(20, channels);
        assertEquals(500, units);

        EMAComponentInstanceSymbol net2Instance = networkInstance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net2", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net2Instance);
        assertEquals( "modularNetworkComplex.connector.predictor1.net2", net2Instance.getFullName());

        EMAComponentInstanceSymbol net3Instance = net2Instance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net3", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net3Instance);
        assertEquals( "modularNetworkComplex.connector.predictor1.net2.net3", net3Instance.getFullName());

        ArchitectureSymbol net3ArchSym = net3Instance.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        assertNotNull(net3ArchSym);
        net3ArchSym.resolve();

        units = ((LayerSymbol)net3ArchSym.getStreams().get(0).getElements().get(1)).getArgument("units").get().getRhs().getIntValue().get();
        assertEquals(10, units);

        EMAComponentInstanceSymbol net4Instance = net2Instance.getSpannedScope().<EMAComponentInstanceSymbol>resolve("net4", EMAComponentInstanceSymbol.KIND).get();
        assertNotNull(net4Instance);
        assertEquals( "modularNetworkComplex.connector.predictor1.net2.net4", net4Instance.getFullName());

        ArchitectureSymbol net4ArchSym = net4Instance.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();
        assertNotNull(net4ArchSym);
        net4ArchSym.resolve();

        Log.info("END", "MODULAR_TEST");
    }
}

