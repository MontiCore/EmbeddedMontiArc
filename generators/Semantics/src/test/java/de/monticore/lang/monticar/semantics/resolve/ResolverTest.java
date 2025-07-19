package de.monticore.lang.monticar.semantics.resolve;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.semantics.construct.SymtabCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.GlobalScope;
import de.se_rwth.commons.logging.Log;
import de.se_rwth.commons.logging.LogStub;
import org.apache.commons.lang3.StringUtils;
import org.junit.Test;

import java.io.IOException;
import java.util.Optional;

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


    private EMAComponentInstanceSymbol testComponent(String model) {
        Log.init();
        Log.enableFailQuick(true);

        EmbeddedMontiArcMathParser parser = new EmbeddedMontiArcMathParser();
        Optional<ASTEMACompilationUnit> ast = null;
        try {
            String componentName = model.replace(".", "/");
            componentName = componentName.substring(0, componentName.lastIndexOf("/"))
                    + "/" + StringUtils.capitalize(componentName.substring(componentName.lastIndexOf("/") + 1));
            ast = parser.parse("src/test/resources/" + componentName + ".emam");
        } catch (IOException e) {
            e.printStackTrace();
        }
        assert (ast.isPresent());

        TaggingResolver symTab = SymtabCreator.createSymTab("src/test/resources", "src/main/resources",
                "target/generated-components");
        LogStub.init();
        Log.enableFailQuick(true);
        Resolver resolver = new Resolver(symTab, model);
        resolver.handleScope();
        EMAComponentInstanceSymbol component = symTab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        assert(component != null);
        return component;
    }
}