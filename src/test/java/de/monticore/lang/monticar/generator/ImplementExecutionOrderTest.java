package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * TODO
 *
 * @author (last commit) $Author$
 * @version $Revision$, $Date$
 * @since TODO: add version number
 */
public class ImplementExecutionOrderTest extends AbstractSymtabTest {

    @Test
    public void testVelocityControlExecOrder() throws Exception {
        Scope symTab = createSymTab("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "fas.demo_fas_Fkt_m.velocityControl", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        System.out.println("Before inst: " + inst);
        List<ExpandedComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(inst);
        System.out.println("After inst: " + inst);
        assertEquals(13, exOrder.size());
        System.out.println(exOrder);
        assertEquals(exOrder.get(0).getName(), "sat1");
        assertEquals(exOrder.get(1).getName(), "look1");
        assertEquals(exOrder.get(2).getName(), "sat2");
        assertEquals(exOrder.get(3).getName(), "look2");
        assertEquals(exOrder.get(4).getName(), "greater1");
        assertEquals(exOrder.get(5).getName(), "max1");
        assertEquals(exOrder.get(6).getName(), "greater2");
        assertEquals(exOrder.get(7).getName(), "and1");
        assertEquals(exOrder.get(8).getName(), "switch2");
        assertEquals(exOrder.get(9).getName(), "max2");
        assertEquals(exOrder.get(10).getName(), "switch1");
        assertEquals(exOrder.get(11).getName(), "mul");
        assertEquals(exOrder.get(12).getName(), "switch3");
    }

    @Test
    public void testRSFlipFlopExecOrder() throws Exception {
        Scope symTab = createSymTab("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "fas.advancedLibrary.rSFlipFlop", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        List<ExpandedComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(inst);
        assertEquals(6, exOrder.size());
        assertEquals(exOrder.get(0).getName(), "oneS");
        assertEquals(exOrder.get(1).getName(), "zeroR");
        assertEquals(exOrder.get(2).getName(), "memory_Q");
        assertEquals(exOrder.get(3).getName(), "switch_S");
        assertEquals(exOrder.get(4).getName(), "switch_R");
        assertEquals(exOrder.get(5).getName(), "logOp_N");
    }

    @Test
    public void testMainControllerExecOrder() throws Exception {
        Scope symTab = createSymTab("src/test/resources/streams", "src/test/resources");
        ExpandedComponentInstanceSymbol inst = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "simulator.mainController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        List<ExpandedComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(inst);
        Log.info(exOrder.size() + "", "size:");

        for (ExpandedComponentInstanceSymbol instanceSymbol : exOrder)
            Log.info(instanceSymbol.getName(), "Name:");
       /* assertEquals(6, exOrder.size());
        assertEquals(exOrder.get(0).getName(), "oneS");
        assertEquals(exOrder.get(0).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:0]");
        assertEquals(exOrder.get(1).getName(), "zeroR");
        assertEquals(exOrder.get(1).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:1]");
        assertEquals(exOrder.get(2).getName(), "memory_Q");
        assertEquals(exOrder.get(2).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:2]");
        assertEquals(exOrder.get(3).getName(), "switch_S");
        assertEquals(exOrder.get(3).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:3]");
        assertEquals(exOrder.get(4).getName(), "switch_R");
        assertEquals(exOrder.get(4).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:4]");
        assertEquals(exOrder.get(5).getName(), "logOp_N");
        assertEquals(exOrder.get(5).getTags(TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:5]");
    */
    }

}
