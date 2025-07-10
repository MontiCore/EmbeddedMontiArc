/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.order.tools.Slist;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.prettyprint.IndentPrinter;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * Created by ernst on 13.08.2016.
 */
public class SListTest extends AbstractSymtabTest {

    @Test
    public void testSList() {
        Slist.resetNonVirtualBlockSize();
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "fas.advancedLibrary.rSFlipFlop", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        IndentPrinter ip = new IndentPrinter();
        ip.println("---- Sorted list for 'rSFlipFlop' [6 nonvirtual block(s), directfeed=0]");
        ip.indent();
        ip.println("0:0    'fas.advancedLibrary.rSFlipFlop.oneS' (Constant, tid=PRM)");
        ip.println("0:1    'fas.advancedLibrary.rSFlipFlop.zeroR' (Constant, tid=PRM)");
        ip.println("0:2    'fas.advancedLibrary.rSFlipFlop.memory_Q' (Memory, tid=0)");
        ip.println("0:3    'fas.advancedLibrary.rSFlipFlop.switch_S' (SwitchB, tid=0)");
        ip.println("0:4    'fas.advancedLibrary.rSFlipFlop.switch_R' (SwitchB, tid=0)");
        ip.println("0:5    'fas.advancedLibrary.rSFlipFlop.logOp_N' (Not, tid=0)");
        ip.unindent();
        assertEquals(ip.getContent(), Slist.execute(symTab, inst));
        Log.info(Slist.execute(symTab, inst), "SLIST");
        Log.info("", "testSList Done");
    }

    @Test
    public void testSlistDetection() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "detection.objectDetector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (EMAComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }

    @Test
    public void testSlistMatrixModifier() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "paper.matrixModifier", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (EMAComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }

    @Test
    public void testSlistMathUnit() {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources/streams", "src/test/resources");
        EMAComponentInstanceSymbol inst = symTab.<EMAComponentInstanceSymbol>resolve(
                "paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(inst);
        Log.info(inst.toString(), "Component:");
        Log.info(Slist.execute(symTab, inst), "SLIST");
        for (EMAComponentInstanceSymbol symbol : inst.getIndependentSubComponents())
            Log.info(symbol.getName(), "Independent comps");

    }
}
