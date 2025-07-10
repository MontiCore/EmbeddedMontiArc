/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.dev;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.StreamScanner;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.ParserTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.order.simulator.AbstractSymtab;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.lang3.SystemUtils;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class Parser extends ParserTest{

    public static final boolean ENABLE_FAIL_QUICK = false;

    @Ignore
    @Test
    public void dev1() throws Exception {
        test("emam", "src/test/resources/fas/basicLibrary");
        if (Log.getErrorCount() > 0) {
            throw new Exception("Test Failed, found errors");
        }
    }

    private void test(String fileEnding, String path) throws IOException {
        ParserTest.ParseTest parserTest = new ParseTest("." + fileEnding);
        Files.walkFileTree(Paths.get(path), parserTest);

        if (!parserTest.getModelsInError().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getModelsInError()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserTest");
        Log.info("Count of correctly parsed models: "
                + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserTest");

        assertTrue("There were models that could not be parsed", parserTest.getModelsInError()
                .isEmpty());
    }


    @Test
    public void Dev_1_Test(){
        TaggingResolver symTab = AbstractSymtab.createSymTabAndTaggingResolver("src/test/resources/emastudio/autopilot");
        ComponentStreamUnitsSymbol componentSymbol = symTab.<ComponentStreamUnitsSymbol>resolve(
                "de.rwth.armin.modeling.autopilot.motion.SteeringAngleCorrection",
                ComponentStreamUnitsSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
    }


    @Ignore
    @Test
    public void Dev_2_Test(){
        TaggingResolver symTab = AbstractSymtab.createSymTabAndTaggingResolver("src/test/resources");
        StreamScanner scanner = new StreamScanner(Paths.get("src/test/resources"), symTab);
        Map<EMAComponentSymbol, Set<ComponentStreamUnitsSymbol>> availableStreams = new HashMap<>(scanner.scan());

//        System.out.println(availableStreams);
    }

    @Ignore
    @Test
    public void Dev_3_Test() throws IOException {
        TaggingResolver symTab = AbstractSymtab.createSymTabAndTaggingResolver("src/test/resources/emastudio/autopilot");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve("de.rwth.armin.modeling.autopilot.autopilot", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/dev-test/test-3/");
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources/emastudio/autopilot/"));
        generatorCPP.setGenerateTests(true);
        generatorCPP.setCheckModelDir(true);
        List<File> files = generatorCPP.generateFiles(symTab, componentSymbol);;
//        System.out.println(files);
    }
}
