/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.monticar.semantics.util.PrintExecutionOrder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

public class ExecutionSemanticsTest {

    @Before
    public void init(){
        Log.initWARN();
        Log.enableFailQuick(true);
    }


    @Test
    public void testSimpleLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.simpleLoop");
    }

    @Test
    public void testSerialLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.serialLoop");
    }

    @Test
    public void testParallelLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.parallelLoop");
    }

    @Test
    public void test02() {
        testComponent("de.monticore.lang.monticar.semantics.loops.test02");
    }

    @Test
    public void testOscillation() {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillation");
    }

    @Test
    public void testOscillationAsSymbol() {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillationAsSymbol");
    }

    @Test
    public void testUnderSpecification() {
        testComponent("de.monticore.lang.monticar.semantics.loops.underSpecification");
    }

    @Test
    public void testNondirectFeedthrough() {
        testComponent("de.monticore.lang.monticar.semantics.loops.delayLoop");
    }

    @Test
    public void testArtificial() {
        EMAComponentInstanceSymbol component = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.artificial");
        EMAComponentInstanceSymbol component2 = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.nVComp");
    }

    @Test
    public void testArtificialDelay() {
        EMAComponentInstanceSymbol component = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.artificialDelay");
        EMAComponentInstanceSymbol component2 = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.nVCompDelay");
    }

    @Test
    public void testAutopilot() {
        EMAComponentInstanceSymbol component = testComponent("de.rwth.armin.modeling.autopilot.autopilot");
    }


    private EMAComponentInstanceSymbol testComponent(String model) {

        TaggingResolver symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                Constants.SYNTHESIZED_COMPONENTS_ROOT);

        EMAComponentInstanceSymbol component =
                symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);

        ExecutionSemantics executionSemantics = new ExecutionSemantics(symTab, component);

        executionSemantics.setResolveLoops(true);
        executionSemantics.setHandleArtificialLoops(true);
        executionSemantics.setSolveSymbolicLoops(true);
        executionSemantics.setSolveSymbolicSpecification(true);

        executionSemantics.setWarnLoops(false);
        executionSemantics.setWarnArtificialLoops(false);
        executionSemantics.setLogSymbolicSolve(false);

        executionSemantics.addExecutionSemantics();






        System.out.println("\n\nExecution Order of model \"" + model + "\": \n" + PrintExecutionOrder.printExecutionOrder(component));
        System.out.println("\nSList of model \"" + model + "\": \n" + PrintExecutionOrder.printSListSerial(component));

        return symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
    }
}
