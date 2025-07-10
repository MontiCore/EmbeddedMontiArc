/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.application;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;
import de.monticore.symboltable.Scope;


public class SimulatorTest extends AbstractSymtabTest{

    @Ignore
    @Test
    public void resolveTest() {
        Scope symtab = createSymTab("src/test/resources");
        ExpandedComponentInstanceSymbol model = symtab.
                <ExpandedComponentInstanceSymbol>resolve("simulator.mainController",
                        ExpandedComponentInstanceSymbol.KIND).orElse(null);

        Assert.assertNotNull(model);
    }
}
