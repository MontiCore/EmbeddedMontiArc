package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.junit.Test;

public class ResolverTest {

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

    private EMAComponentInstanceSymbol testComponent(String model) {
        Log.init();
        Log.enableFailQuick(true);
        GlobalScope symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                "target/generated-components");
        LogStub.init();
        Log.enableFailQuick(true);
        Resolver resolver = new Resolver(model);
        resolver.handleScope(symTab);
        EMAComponentInstanceSymbol component = symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        assert(component != null);
        return component;
    }
}