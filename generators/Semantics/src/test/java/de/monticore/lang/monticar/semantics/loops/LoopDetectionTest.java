package de.monticore.lang.monticar.semantics.loops;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.loops.detection.Detection;
import de.monticore.lang.monticar.semantics.loops.detection.StrongConnectedComponent;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.Set;

public class LoopDetectionTest extends AbstractSymtabTest{

    @Before
    public void init() {
        Log.init();
    }

    @Test
    public void test1() {
        check("de.monticore.lang.monticar.semantics.loops.test01");
    }

    @Test
    public void test2() {
        check("de.monticore.lang.monticar.semantics.loops.serialLoop");
    }

    @Test
    public void test3() {
        check("de.monticore.lang.monticar.semantics.loops.parallelLoop");
    }

    @Test
    public void test4() {
        check("de.monticore.lang.monticar.semantics.loops.vanDerPolEquation");
    }

    @Test
    public void test5() {
        check("de.monticore.lang.monticar.semantics.loops.simulinkExample2");
    }

    @Test
    public void test6() {
        check("de.monticore.lang.monticar.semantics.loops.simpleLoop");
    }

    public void check(String model) {
        Scope symTab = createSymTab("src/test/resources", "src/main/resources");
        EMAComponentInstanceSymbol component = symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        Set<StrongConnectedComponent> strongConnectedComponents = Detection.detectLoops(component);
    }
}
