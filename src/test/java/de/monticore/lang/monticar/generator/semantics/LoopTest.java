/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.semantics;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._parser.EmbeddedMontiArcMathParser;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.semantics.ExecutionSemantics;
import de.monticore.lang.monticar.semantics.helper.NameHelper;
import de.monticore.lang.monticar.semantics.resolve.Resolver;
import de.monticore.lang.monticar.semantics.util.BasicLibrary;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.StringUtils;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab.createSymTabAndTaggingResolver;
import static org.junit.Assert.assertNotNull;

public class LoopTest {

    @Test
    public void testSimpleLoop() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.simpleLoop");
    }

    @Test
    public void testSerialLoop() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.serialLoop");
    }

    @Test
    public void testParallelLoop() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.parallelLoop");
    }

    @Ignore
    @Test
    public void test02() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.test02");
    }

    @Test
    public void testOscillation() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillation");
    }

    @Test
    public void testOscillationAsDAESymbol() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillationAsSymbol");
    }

    @Test
    public void testOscillationAsODESymbol() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.oscillationAsODESymbol");
    }

    @Ignore
    @Test
    public void testUnderSpecification() throws IOException {
        testComponent("de.monticore.lang.monticar.semantics.loops.underSpecification");
    }

    private EMAComponentInstanceSymbol testComponent(String model) throws IOException {
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

        BasicLibrary.extract();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources", "src/main/resources",
                BasicLibrary.BASIC_LIBRARY_ROOT);

        Optional<EMAComponentInstanceSymbol> resolve = symtab.resolve(model, EMAComponentInstanceSymbol.KIND);


        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve(model, EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/" + NameHelper.toInstanceName(model));
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setDeltaT(0.1);
        generatorCPP.setResolveLoops(true);
        generatorCPP.setSolveLoopsSymbolic(false);
        generatorCPP.setGenerateCMake(true);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);

        return componentSymbol;
    }
}
