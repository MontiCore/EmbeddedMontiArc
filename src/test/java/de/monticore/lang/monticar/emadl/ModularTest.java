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
import org.junit.Test;

import java.util.Optional;

import static junit.framework.TestCase.assertEquals;

public class ModularTest extends AbstractSymtabTest{

    @Before
    public void setUp() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testInstances(){
        Scope symtab = createSymTab("src/test/resources/models/");
        Optional<EMAComponentInstanceSymbol> compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("InstanceTest.mainB", EMAComponentInstanceSymbol.KIND);
        if(compInstanceSymbol.isPresent()){
            EMAComponentInstanceSymbol mainInstance = compInstanceSymbol.get();
        }


        /*
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
        */
    }

    @Test
    public void testModular(){
        Scope symtab = createSymTab("src/test/resources/models/ModularMNIST/");
        Optional<EMAComponentInstanceSymbol> compInstanceSymbol = symtab.<EMAComponentInstanceSymbol>resolve("modularNetworks.Connector", EMAComponentInstanceSymbol.KIND);
        if(compInstanceSymbol.isPresent()){
            EMAComponentInstanceSymbol mainInstance = compInstanceSymbol.get();
        }
    }
}

