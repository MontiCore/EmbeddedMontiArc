/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import java.util.*;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.ImplementExecutionOrder;
import de.monticore.lang.monticar.generator.order.nfp.TagExecutionOrderTagSchema.TagExecutionOrderSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;
import de.monticore.symboltable.Scope;

/**
 * TODO
 *
 */
public class ImplementExecutionOrderTest extends AbstractSymtabTest {

    @Test
    public void testVelocityControlExecOrder() throws Exception {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "fas.demo_fas_Fkt_m.velocityControl", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
//        System.out.println("Before inst: " + inst);
        List<EMAComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(symTab, inst);
//        System.out.println("After inst: " + inst);
        assertEquals(13, exOrder.size());
//        System.out.println(exOrder);
        assertEquals(exOrder.get(0).getName(), "sat1");
        assertEquals(symTab.getTags(exOrder.get(0), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:0]");
        assertEquals(exOrder.get(1).getName(), "look1");
        assertEquals(symTab.getTags(exOrder.get(1), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:1]");
        assertEquals(exOrder.get(2).getName(), "sat2");
        assertEquals(symTab.getTags(exOrder.get(2), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:2]");
        assertEquals(exOrder.get(3).getName(), "look2");
        assertEquals(symTab.getTags(exOrder.get(3), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:3]");
        assertEquals(exOrder.get(4).getName(), "greater1");
        assertEquals(symTab.getTags(exOrder.get(4), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:4]");
        assertEquals(exOrder.get(5).getName(), "max1");
        assertEquals(symTab.getTags(exOrder.get(5), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:5]");
        assertEquals(exOrder.get(6).getName(), "greater2");
        assertEquals(symTab.getTags(exOrder.get(6), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:6]");
        assertEquals(exOrder.get(7).getName(), "and1");
        assertEquals(symTab.getTags(exOrder.get(7), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:7]");
        assertEquals(exOrder.get(8).getName(), "switch2");
        assertEquals(symTab.getTags(exOrder.get(8), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:8]");
        assertEquals(exOrder.get(9).getName(), "max2");
        assertEquals(symTab.getTags(exOrder.get(9), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:9]");
        assertEquals(exOrder.get(10).getName(), "switch1");
        assertEquals(symTab.getTags(exOrder.get(10), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:10]");
        assertEquals(exOrder.get(11).getName(), "mul");
        assertEquals(symTab.getTags(exOrder.get(11), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:11]");
        assertEquals(exOrder.get(12).getName(), "switch3");
        assertEquals(symTab.getTags(exOrder.get(12), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:12]");
    }

    @Test
    public void testRSFlipFlopExecOrder() throws Exception {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "fas.advancedLibrary.rSFlipFlop", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        List<EMAComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(symTab, inst);
        assertEquals(6, exOrder.size());
        assertEquals(exOrder.get(0).getName(), "oneS");
        assertEquals(symTab.getTags(exOrder.get(0), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:0]");
        assertEquals(exOrder.get(1).getName(), "zeroR");
        assertEquals(symTab.getTags(exOrder.get(1), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:1]");
        assertEquals(exOrder.get(2).getName(), "memory_Q");
        assertEquals(symTab.getTags(exOrder.get(2), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:2]");
        assertEquals(exOrder.get(3).getName(), "switch_S");
        assertEquals(symTab.getTags(exOrder.get(3), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:3]");
        assertEquals(exOrder.get(4).getName(), "switch_R");
        assertEquals(symTab.getTags(exOrder.get(4), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:4]");
        assertEquals(exOrder.get(5).getName(), "logOp_N");
        assertEquals(symTab.getTags(exOrder.get(5), TagExecutionOrderSymbol.KIND).toString(), "[TagExecutionOrder = 0:5]");
    }

    @Test
    public void testMainControllerExecOrder() throws Exception {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "simulator.mainController", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        List<EMAComponentInstanceSymbol> exOrder = ImplementExecutionOrder.exOrder(symTab, inst);
        Log.info(exOrder.size() + "", "size:");

        for (EMAComponentInstanceSymbol instanceSymbol : exOrder)
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
