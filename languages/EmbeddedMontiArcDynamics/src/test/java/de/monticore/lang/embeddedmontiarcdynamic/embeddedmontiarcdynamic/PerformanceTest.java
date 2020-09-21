package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;


import de.monticore.lang.embeddedmontiarcdynamic.consistencychecks.ConsistencyChecker;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.util.Date;
import java.util.Optional;

public class PerformanceTest extends AbstractTest {

    Scope symScope = createSymTab("src/test/resources/test/embeddedmontiarcdynamic/");


    @Test
    public void PerformanceTest_01_TS() {
        runPerformanceTest("performance.TS");
    }

    @Test
    public void PerformanceTest_02_TP() {
        runPerformanceTest("performance.TP");
    }

    @Test
    public void PerformanceTest_03_TPC() {
        runPerformanceTest("performance.TPC");
    }

    @Test
    public void PerformanceTest_04_TPE() {
        runPerformanceTest("performance.TPE");
    }

    @Test
    public void PerformanceTest_05_TPI() {
        runPerformanceTest("performance.TPI");
    }

    @Test
    public void PerformanceTest_06_TPEC() {
        runPerformanceTest("performance.TPEC");
    }

    @Test
    public void PerformanceTest_07_TPIC() {
        runPerformanceTest("performance.TPIC");
    }

    private void runPerformanceTest(String componentName) {
        Date durationResolveSymTab = new Date();
        Optional<EMADynamicComponentSymbol> comp = symScope.resolve(componentName, EMADynamicComponentSymbol.KIND);

        Log.info(String.format("Generation of symbol table: %1$s seconds", (1.0 * (new Date().getTime() - durationResolveSymTab.getTime()) / 1000)), "Performance");

        ConsistencyChecker consistencyChecker = new ConsistencyChecker();
        if (!comp.isPresent()) {
            Log.error(
                    "Component not available: " + componentName
            );
            return;
        }
        consistencyChecker.CheckWithPerformance(comp.get());
    }

}
