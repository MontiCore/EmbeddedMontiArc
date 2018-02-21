package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.apache.commons.cli.Option;
import org.junit.Assert;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

public class GeneratorCppCliTest extends AbstractSymtabTest {

    @Test
    public void testInstructionsOrder1() throws IOException {
        generateAndCompareMyComponentX(8);
    }

    @Test
    public void testInstructionsOrder2() throws IOException {
        generateAndCompareMyComponentX(9);
    }

    private void generateAndCompareMyComponentX(int x) throws IOException {
        generateAndCompare(
                String.format("testing.subpackage%1$s.myComponent%1$s", x),
                String.format("./target/generated-sources-cpp/testing/MyComponent%s", x),
                String.format("testing/MyComponent%s/", x)
        );
    }

    private void generateAndCompare(String component, String targetPath, String pathToExpectedResults) throws IOException {
        String[] args = new String[]{
                formatParam(GeneratorCppCli.OPTION_MODELS_PATH, "src/test/resources"),
                formatParam(GeneratorCppCli.OPTION_ROOT_MODEL, component),
                formatParam(GeneratorCppCli.OPTION_OUTPUT_PATH, targetPath),
        };
        GeneratorCppCli.main(args);
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentSymbol = symTab.<ExpandedComponentInstanceSymbol>resolve(
                component,
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath(targetPath);
        List<File> files = generatorCPP.generateFiles(componentSymbol, symTab);
        Assert.assertNotNull(files);
        Assert.assertFalse(files.isEmpty());
        testFilesAreEqual(files, pathToExpectedResults);
    }

    private static String formatParam(Option param, String value) {
        return String.format("--%s=%s", param.getLongOpt(), value);
    }
}
