/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics;

import de.monticore.ModelingLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.EmbeddedMontiArcLanguage;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.monticar.semantics.helper.Find;
import de.monticore.lang.monticar.semantics.util.PrintExecutionOrder;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.util.Comparator;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

public class ExecutionSemanticsTest {

    @Before
    public void init() {
        Log.initWARN();
        Log.enableFailQuick(true);
    }


    @Test
    public void testSimpleLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.simpleLoop", null);
    }

    @Test
    public void testSerialLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.serialLoop", null);
    }

    @Test
    public void testParallelLoop() {
        testComponent("de.monticore.lang.monticar.semantics.loops.parallelLoop", null);
    }

    @Test
    public void test02() {
        testComponent("de.monticore.lang.monticar.semantics.loops.test02", null);
    }

    @Test
    public void testOscillation() {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillation", null);
    }

    @Test
    public void testOscillationAsSymbol() {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillationAsSymbol", null);
    }

    @Test
    public void testUnderSpecification() {
        testComponent("de.monticore.lang.monticar.semantics.loops.underSpecification", null);
    }

    @Test
    public void testNondirectFeedthrough() {
        testComponent("de.monticore.lang.monticar.semantics.loops.delayLoop", null);
    }

    @Test
    public void testArtificial() {
        EMAComponentInstanceSymbol component = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.artificial", null);
        EMAComponentInstanceSymbol component2 = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.nVComp", null);
    }

    @Test
    public void testArtificialDelay() {
        EMAComponentInstanceSymbol component = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.artificialDelay", null);
        EMAComponentInstanceSymbol component2 = testComponent("de.monticore.lang.monticar.semantics.loops.artificial.nVCompDelay", null);
    }

    @Test
    public void testAutopilot() {
        EMAComponentInstanceSymbol component = testComponent("de.rwth.armin.modeling.autopilot.autopilot", null);
    }


    private EMAComponentInstanceSymbol testComponent(final String model, final ModelingLanguage modelingLanguage) {

        final TaggingResolver symTab;
        if (modelingLanguage == null) {
            symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                    Constants.SYNTHESIZED_COMPONENTS_ROOT);
        } else {
            symTab = SymtabCreator.createSymTabForLanguage(modelingLanguage, "src/test/resources", "src/main/resources",
                    Constants.SYNTHESIZED_COMPONENTS_ROOT);
        }
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

    @Test
    public void correctOrderForMachineLearningPipeline() {
        final EMAComponentInstanceSymbol component = testComponent("de.monticore.lang.ema.pipeline", new EmbeddedMontiArcLanguage());
        final List<EMAComponentInstanceSymbol> emaComponents = Find.allAtomicOrNVComponents(component);
        sortcomponentsByExecutionOrder(emaComponents);
        final String[] expectedOrder = new String[]{"data_access_step", "training_step", "evaluation_step"};
        final String[] actualOrder = emaComponents.stream().map(CommonSymbol::getName).toArray(String[]::new);
        assertArrayEquals(expectedOrder, actualOrder, "Pipeline execution order is not as expected");
    }

    private void sortcomponentsByExecutionOrder(List<EMAComponentInstanceSymbol> emaComponents) {
        Comparator<EMAComponentInstanceSymbol> comparator = (EMAComponentInstanceSymbol comp1, EMAComponentInstanceSymbol comp2) -> Integer.compareUnsigned(comp1.getOrderOutput().get(0), comp2.getOrderOutput().get(0));
        emaComponents.sort(comparator);
    }
}
