package de.monticore.lang.monticar.semantics.loops;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.LoopDetection;
import de.monticore.lang.monticar.semantics.loops.detection.SimpleCycle;
import de.monticore.lang.monticar.semantics.loops.detection.StronglyConnectedComponent;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.Set;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class LoopDetectionTest extends AbstractSymtabTest{

    @Before
    public void init() {
        Log.initWARN();
    }

    @Test
    public void test1() {
        getLoops("de.monticore.lang.monticar.semantics.loops.test01");
    }

    @Test
    public void test2() {
        Set<StronglyConnectedComponent> loops = getLoops("de.monticore.lang.monticar.semantics.loops.serialLoop");
        assertEquals(1, loops.size());
        assertEquals(3, loops.stream().findFirst().get().getSimpleCycles().size());
    }

    @Test
    public void test3() {
        Set<StronglyConnectedComponent> loops = getLoops("de.monticore.lang.monticar.semantics.loops.parallelLoop");
        assertEquals(1, loops.size());
        assertEquals(4, loops.stream().findFirst().get().getSimpleCycles().size());
    }

    @Test
    public void test4() {
        getLoops("de.monticore.lang.monticar.semantics.loops.vanDerPolEquation");
    }

    @Test
    public void test5() {
        getLoops("de.monticore.lang.monticar.semantics.loops.simulinkExample2");
    }

    @Test
    public void test6() {
        Set<StronglyConnectedComponent> loops = getLoops("de.monticore.lang.monticar.semantics.loops.simpleLoop");
        assertEquals(1, loops.size());
        assertEquals(1, loops.stream().findFirst().get().getSimpleCycles().size());
    }

    @Test
    public void testNondirectFeedthrough() {
        String model = "de.monticore.lang.monticar.semantics.loops.delayLoop";
        Set<StronglyConnectedComponent> stronglyConnectedComponents =
                getLoops("de.monticore.lang.monticar.semantics.loops.delayLoop");
        assertEquals (0, stronglyConnectedComponents.size());
    }

    @Test
    public void testArtificial() {
        Set<StronglyConnectedComponent> stronglyConnectedComponents =
                getLoops("de.monticore.lang.monticar.semantics.loops.artificial.artificial");
        assertEquals (1, stronglyConnectedComponents.size());
        for (StronglyConnectedComponent stronglyConnectedComponent : stronglyConnectedComponents) {
            assertTrue (stronglyConnectedComponent.isArtificial());
            for (SimpleCycle simpleCycle : stronglyConnectedComponent.getSimpleCycles()) {
                assertTrue (simpleCycle.isArtificial());
            }
        }
    }

    public Set<StronglyConnectedComponent> getLoops(String model) {
        Scope symTab = createSymTab("src/test/resources", "src/main/resources");
        EMAComponentInstanceSymbol component = symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        Set<StronglyConnectedComponent> stronglyConnectedComponents = LoopDetection.detectLoops(component);
        return stronglyConnectedComponents;
    }
}
