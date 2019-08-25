/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.symboltable.Scope;
import org.junit.Ignore;
import org.junit.Test;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertNotNull;

public class ConnectorGetPortTest extends AbstractSymtabTest {

    @Ignore
    @Test
    public void testgetPortsList() throws Exception {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "fas.demo_fas_Fkt_m.fAS", EMAComponentInstanceSymbol.KIND).orElse(null);

        testConnectorPorts(inst);
    }

    private void testConnectorPorts(EMAComponentInstanceSymbol inst) {
        assertNotNull(inst);

        inst.getConnectorInstances().forEach(con -> {
            assertNotEquals(con.getSource(), con.getTarget());
            assertNotEquals(con.getSourcePort().getFullName(), con.getTargetPort().getFullName());
        });

        inst.getSubComponents().forEach(this::testConnectorPorts);
    }

}
