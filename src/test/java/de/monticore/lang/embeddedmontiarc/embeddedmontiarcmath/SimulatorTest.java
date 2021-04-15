/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
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
