package de.monticore.lang.monticar.semantics.loops;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.Detection;
import de.monticore.lang.monticar.semantics.loops.detection.EMAGraphTransformation;
import de.monticore.lang.monticar.semantics.loops.graph.EMAGraph;
import de.monticore.lang.monticar.semantics.loops.graph.EMAVertex;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.junit.Before;
import org.junit.Test;

import java.util.List;
import java.util.Map;
import java.util.Set;

public class LoopDetectionTest extends AbstractSymtabTest{

    @Before
    public void init() {
        Log.init();
    }

    @Test
    public void test1() {
        check("de.monticore.lang.monticar.semantics.loops.Test01");
    }

    @Test
    public void test2() {
        check("de.monticore.lang.monticar.semantics.loops.SerialLoop");
    }

    @Test
    public void test3() {
        check("de.monticore.lang.monticar.semantics.loops.ParallelLoop");
    }

    @Test
    public void test4() {
        check("de.monticore.lang.monticar.semantics.loops.VanDerPolEquation");
    }

    @Test
    public void test5() {
        check("de.monticore.lang.monticar.semantics.loops.SimulinkExample2");
    }

    public void check(String model) {
        Scope symTab = createSymTab("src/test/resources");
        EMAComponentSymbol component = symTab.<EMAComponentSymbol>resolve(model, EMAComponentSymbol.KIND).orElse(null);
        Detection detection = new Detection();
        Detection.Loops loops = detection.detectLoops(component);
        Map<Set<EMAVertex>, List<List<EMAVertex>>> cyclesForComponent = loops.getCyclesForComponent();
        cyclesForComponent.size();
    }
}
