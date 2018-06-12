package de.monticore.lang.monticar.generator.cpp.armadillo;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.TestConverter;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha Schneiders
 */
public class ArmadilloFunctionTest extends AbstractSymtabTest {

    public void testMathCommand(String namePart) throws IOException {
        MathConverter.resetIDs();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math." + namePart + "CommandTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    public void testMathCommandStream(String namePart, String streamName) throws IOException {
        MathConverter.resetIDs();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math." + namePart + "CommandTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/armadillo/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "armadillo/testMath/l0/";

        ComponentStreamUnitsSymbol streamSymbol = symtab.<ComponentStreamUnitsSymbol>resolve("test.math." + streamName, ComponentStreamUnitsSymbol.KIND).orElse(null);
        Log.info(streamName, "Resolving:");
        assertNotNull(streamSymbol);
        if (streamSymbol != null) {
            files.add(generatorCPP.generateFile(TestConverter.generateMainTestFile(streamSymbol, componentSymbol)));
        }
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testOnesCommand() throws IOException {
        testMathCommand("ones");
    }

    @Test
    public void testZerosCommand() throws IOException {
        testMathCommand("zeros");
    }

    @Test
    public void testOnesArmadilloOnlyCommand() throws IOException {
        testMathCommand("onesArmadilloOnly");
    }

    @Test
    public void testZerosArmadilloOnlyCommand() throws IOException {
        testMathCommand("zerosArmadilloOnly");
    }

    @Test
    public void testAbsCommand() throws IOException {
        testMathCommand("abs");
    }

    @Test
    public void testAcosCommand() throws IOException {
        testMathCommand("acos");
    }

    @Test
    public void testAcoshCommand() throws IOException {
        testMathCommand("acosh");
    }

    @Test
    public void testAsinCommand() throws IOException {
        testMathCommand("asin");
    }

    @Test
    public void testAsinhCommand() throws IOException {
        testMathCommand("asinh");
    }

    @Test
    public void testAtan2Command() throws IOException {
        testMathCommand("atan2");
    }

    @Test
    public void testAtanCommand() throws IOException {
        testMathCommand("atan");
    }

    @Test
    public void testAtanhCommand() throws IOException {
        testMathCommand("atanh");
    }

    @Test
    public void testCosCommand() throws IOException {
        testMathCommand("cos");
    }


    @Test
    public void testCoshCommand() throws IOException {
        testMathCommand("cosh");
    }

    @Test
    public void testDetCommand() throws IOException {
        testMathCommand("det");
    }

    @Test
    public void testDiagCommand() throws IOException {
        testMathCommand("diag");
    }

    @Test
    public void testEyeCommand() throws IOException {
        testMathCommand("eye");
    }

    @Test
    public void testLog2Command() throws IOException {
        testMathCommand("log2");
    }

    @Test
    public void testLog10Command() throws IOException {
        testMathCommand("log10");
    }

    @Test
    public void testLogCommand() throws IOException {
        testMathCommand("log");
    }

    @Test
    public void testNormCommand() throws IOException {
        testMathCommand("norm");
    }

    @Test
    public void testSinCommand() throws IOException {
        testMathCommand("sin");
    }

    @Test
    public void testSinhCommand() throws IOException {
        testMathCommand("sinh");
    }

    @Test
    public void testTanCommand() throws IOException {
        testMathCommand("tan");
    }

    @Test
    public void testTanhCommand() throws IOException {
        testMathCommand("tanh");
    }
}
