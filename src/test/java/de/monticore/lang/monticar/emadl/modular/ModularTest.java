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

import java.util.Optional;

import static junit.framework.Assert.assertTrue;
import static junit.framework.TestCase.assertEquals;

public class ModularTest extends AbstractSymtabTest {

    @Before
    public void setUp() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testModularInstances(){
        Scope symtab = createSymTab("src/test/resources/models/");
        Optional<EMAComponentInstanceSymbol> compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("ModularInstanceTest.mainB", EMAComponentInstanceSymbol.KIND);

        assertTrue(compInstanceSymbol.isPresent());

        if(compInstanceSymbol.isPresent()){
            EMAComponentInstanceSymbol mainInstance = compInstanceSymbol.get();
            /*
            EMAComponentInstanceSymbol netC = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("netC", EMAComponentInstanceSymbol.KIND).get();
            EMAComponentInstanceSymbol netD = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("netD", EMAComponentInstanceSymbol.KIND).get();

            assertEquals("ModularInstanceTest.mainB.netC",netC.getFullName());
            assertEquals("ModularInstanceTest.mainB.netD",netD.getFullName());

            ArchitectureSymbol arch1 = netC.getSpannedScope().<ArchitectureSymbol>
                    resolve("", ArchitectureSymbol.KIND).get();

            ArchitectureSymbol arch2 = netD.getSpannedScope().<ArchitectureSymbol>
                    resolve("", ArchitectureSymbol.KIND).get();

            arch1.resolve();
            arch2.resolve();

            int convChannels1 = ((LayerSymbol)arch1.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();
            int convChannels2 = ((LayerSymbol)arch2.getStreams().get(0).getElements().get(1)).getArgument("channels").get().getRhs().getIntValue().get();

            assertEquals(20, convChannels1);
            assertEquals(40, convChannels2);

            */
        }

    }

    @Test
    public void testModularSimple(){
        Scope symtab = createSymTab("src/test/resources/models/ModularMNIST/");
        Optional<EMAComponentInstanceSymbol> compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("modularNetworkSimple.Connector", EMAComponentInstanceSymbol.KIND);

        assertTrue(compInstanceSymbol.isPresent());

        if(compInstanceSymbol.isPresent()){


            EMAComponentInstanceSymbol mainInstance = compInstanceSymbol.get();

            EMAComponentInstanceSymbol network1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("predictor1", EMAComponentInstanceSymbol.KIND).get();

            EMAComponentInstanceSymbol net1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("net1", EMAComponentInstanceSymbol.KIND).get();
            EMAComponentInstanceSymbol net2 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                resolve("net2", EMAComponentInstanceSymbol.KIND).get();

            assertEquals("modularNetworks.Connector.predictor1",network1.getFullName());
            assertEquals("modularNetworks.Connector.predictor1.net1",net1.getFullName());
            assertEquals("modularNetworks.Connector.predictor1.net2",net2.getFullName());

            ArchitectureSymbol network1ArchSym = network1.getSpannedScope().<ArchitectureSymbol>resolve("",ArchitectureSymbol.KIND).get();

            ArchitectureSymbol net1ArchSym = net1.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

            ArchitectureSymbol net2ArchSym = net2.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

            net1ArchSym.resolve();
            net2ArchSym.resolve();
            network1ArchSym.resolve();

        }
        Log.info("END","MODULAR_TEST");
    }

    @Test
    public void testModularComplex(){
        Scope symtab = createSymTab("src/test/resources/models/ModularMNIST/");
        Optional<EMAComponentInstanceSymbol> compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("modularNetworkComplex.Connector", EMAComponentInstanceSymbol.KIND);

        assertTrue(compInstanceSymbol.isPresent());

        if(compInstanceSymbol.isPresent()){

        /*
            EMAComponentInstanceSymbol mainInstance = compInstanceSymbol.get();

            EMAComponentInstanceSymbol network1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("predictor1", EMAComponentInstanceSymbol.KIND).get();

            EMAComponentInstanceSymbol net1 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("net1", EMAComponentInstanceSymbol.KIND).get();
            EMAComponentInstanceSymbol net2 = mainInstance.getSpannedScope().<EMAComponentInstanceSymbol>
                    resolve("net2", EMAComponentInstanceSymbol.KIND).get();

            assertEquals("modularNetworks.Connector.predictor1",network1.getFullName());
            assertEquals("modularNetworks.Connector.predictor1.net1",net1.getFullName());
            assertEquals("modularNetworks.Connector.predictor1.net2",net2.getFullName());

            ArchitectureSymbol network1ArchSym = network1.getSpannedScope().<ArchitectureSymbol>resolve("",ArchitectureSymbol.KIND).get();

            ArchitectureSymbol net1ArchSym = net1.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

            ArchitectureSymbol net2ArchSym = net2.getSpannedScope().<ArchitectureSymbol>resolve("", ArchitectureSymbol.KIND).get();

            net1ArchSym.resolve();
            net2ArchSym.resolve();
            network1ArchSym.resolve();
        */
        }
        Log.info("END","MODULAR_TEST");
    }
}

