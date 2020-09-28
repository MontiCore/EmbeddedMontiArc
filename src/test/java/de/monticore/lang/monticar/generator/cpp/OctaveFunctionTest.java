/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;


import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.cpp.converter.MathConverter;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 */
public class OctaveFunctionTest extends AbstractSymtabTest {

    public void testMathCommand(String namePart) throws IOException {
        MathConverter.resetIDs();
        TaggingResolver symtab= createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math." + namePart + "CommandTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    public void testMathCommandStream(String namePart, String streamName) throws IOException {
        MathConverter.resetIDs();
        TaggingResolver symtab= createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math." + namePart + "CommandTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";

        ComponentStreamUnitsSymbol streamSymbol = symtab.<ComponentStreamUnitsSymbol>resolve("test.math." + streamName, ComponentStreamUnitsSymbol.KIND).orElse(null);
        Log.info(streamName, "Resolving:");
        assertNotNull(streamSymbol);
        if (streamSymbol != null) {
            files.add(generatorCPP.generateFile(TestConverter.generateMainTestFile(streamSymbol, componentSymbol)));
        }
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testAbsCommand() throws IOException {
        /*TaggingResolver symtab= createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.math.absCommandTest", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);*/
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
    public void testEigvalCommand() throws IOException {
        testMathCommand("eigval");
    }

    @Test
    public void testEigvecCommand() throws IOException {
        testMathCommand("eigvec");
    }

    @Test
    public void testExpCommand() throws IOException {
        testMathCommand("exp");
    }

    @Test
    public void testEyeCommand() throws IOException {
        testMathCommand("eye");
    }

    @Test
    public void testGcdCommand() throws IOException {
        testMathCommand("gcd");
    }

    @Test
    public void testInvCommand() throws IOException {
        testMathCommand("inv");
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
    public void testMaxCommand() throws IOException {
        testMathCommand("max");
    }

    @Test
    public void testMinCommand() throws IOException {
        testMathCommand("min");
    }


    @Test
    public void testNormCommand() throws IOException {
        testMathCommand("norm");
    }


    @Test
    public void testOnesCommand() throws IOException {
        testMathCommand("ones");
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
    public void testSqrtCommand() throws IOException {
        testMathCommand("sqrt");
    }

    @Test
    public void testSumCommand() throws IOException {
        //testMathCommand("sum");
        testMathCommandStream("sum", "SumCommandTestStream");
    }

    @Test
    public void testTanCommand() throws IOException {
        testMathCommand("tan");
    }

    @Test
    public void testTanhCommand() throws IOException {
        testMathCommand("tanh");
    }

    @Test
    public void testZerosCommand() throws IOException {
        testMathCommand("zeros");
    }
}
