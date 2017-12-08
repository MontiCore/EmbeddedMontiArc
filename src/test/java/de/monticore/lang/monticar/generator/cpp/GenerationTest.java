package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConstantPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.math.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.Helper;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Assert;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha Schneiders
 */
public class GenerationTest extends AbstractSymtabTest {

    @Ignore
    @Test
    public void testBasicConstantAssignment() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicConstantAssignment", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testConstantAssignment");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testBasicPorts() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPorts", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol));
        assertEquals("#ifndef TEST_BASICPORTS\n" +
                "#define TEST_BASICPORTS\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"octave/oct.h\"\n" +
                "class test_basicPorts{\n" +
                "public:\n" +
                "double in1;\n" +
                "double in2;\n" +
                "double out1;\n" +
                "double out2;\n" +
                "void init()\n" +
                "{\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "out1 = in2;\n" +
                "out2 = in1;\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, symtab));

    }

    @Test
    public void testBasicPortsConstantConnector() {
        ConstantPortSymbol.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPortsConstantConnector", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol));
        assertEquals("#ifndef TEST_BASICPORTSCONSTANTCONNECTOR\n" +
                "#define TEST_BASICPORTSCONSTANTCONNECTOR\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"octave/oct.h\"\n" +
                "class test_basicPortsConstantConnector{\n" +
                "public:\n" +
                "double out1;\n" +
                "bool out2;\n" +
                "bool CONSTANTPORT1;\n" +
                "double CONSTANTPORT2;\n" +
                "void init()\n" +
                "{\n" +
                "this->CONSTANTPORT1 = false;\n" +
                "this->CONSTANTPORT2 = 1;\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "out1 = CONSTANTPORT2;\n" +
                "out2 = CONSTANTPORT1;\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, symtab));

    }


    @Test
    public void testPortsMath() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPortsMath", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        //MathStatementsSymbol mathSymbol = symtab.<MathStatementsSymbol>resolve("test.BasicPortsMath.MathStatements", MathStatementsSymbol.KIND).orElse(null);
        //assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol, mathSymbol));
        assertEquals("#ifndef TEST_BASICPORTSMATH\n" +
                "#define TEST_BASICPORTSMATH\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"octave/oct.h\"\n" +
                "class test_basicPortsMath{\n" +
                "public:\n" +
                "double counter;\n" +
                "double result;\n" +
                "void init()\n" +
                "{\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "if((counter < 0)){\n" +
                "result = 0;\n" +
                "}\n" +
                "else if((counter < 100)){\n" +
                "result = counter;\n" +
                "}\n" +
                "else {\n" +
                "result = 100;\n" +
                "}\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, symtab));

    }

    @Test
    public void testPortsLoop() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPortsLoop", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol, mathSymbol));
        assertEquals("#ifndef TEST_BASICPORTSLOOP\n" +
                "#define TEST_BASICPORTSLOOP\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"octave/oct.h\"\n" +
                "class test_basicPortsLoop{\n" +
                "public:\n" +
                "double counter;\n" +
                "double result;\n" +
                "void init()\n" +
                "{\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "for( auto i=1;i<=8;i+=1){\n" +
                "result = result+counter;\n" +
                "}\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, symtab));

    }

    @Ignore
    @Test
    public void testSimulatorSpeedLimitChecker() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulator.speedLimitChecker", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        MathStatementsSymbol mathSymbol = symtab.<MathStatementsSymbol>resolve("simulator.SpeedLimitChecker.MathStatements", MathStatementsSymbol.KIND).orElse(null);
        assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol, mathSymbol));
        assertEquals("#ifndef SIMULATOR_SPEEDLIMITCHECKER\n" +
                "#define SIMULATOR_SPEEDLIMITCHECKER\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "class simulator_speedLimitChecker{\n" +
                "public:\n" +
                "double currentVelocity;\n" +
                "double currentSpeedLimit;\n" +
                "bool speedLimitSurpassed;\n" +
                "void init()\n" +
                "{\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "if(((currentVelocity > currentSpeedLimit))){\n" +
                "speedLimitSurpassed = true;\n" +
                "}\n" +
                "else {\n" +
                "speedLimitSurpassed = false;\n" +
                "}\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, symtab));

    }

    @Ignore
    @Test
    public void testSimulatorBrakeController() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulator.BrakeController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol, mathSymbol));
        assertEquals("#ifndef SIMULATOR_BRAKECONTROLLER\n" +
                "#define SIMULATOR_BRAKECONTROLLER\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"simulator_SpeedLimitChecker.h\"\n" +
                "class simulator_BrakeController{\n" +
                "public:\n" +
                "double currentSpeedLimit;\n" +
                "double currentVelocity;\n" +
                "double brakeForce;\n" +
                "simulator_SpeedLimitChecker speedLimitChecker1;\n" +
                "void setInputs(double currentSpeedLimit, double currentVelocity)\n" +
                "{\n" +
                "this->currentSpeedLimit = currentSpeedLimit;\n" +
                "this->currentVelocity = currentVelocity;\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "if (speedLimitChecker1.speedLimitSurpassed){\n" +
                "(brakeForce = 0.5);\n" +
                "}\n" +
                "else{\n" +
                "(brakeForce = 0);\n" +
                "}\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, mathSymbol));

    }

    @Ignore
    @Test
    public void testSimulatorSteerController() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources/simulator");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("steerController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        //System.out.println(generatorCPP.generateString(componentSymbol, mathSymbol));
        assertEquals("#ifndef SIMULATOR_STEERCONTROLLER\n" +
                "#define SIMULATOR_STEERCONTROLLER\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "class simulator_steerController{\n" +
                "public:\n" +
                "double currentSteeringAngle;\n" +
                "double steeringAngle;\n" +
                "void setInputs(double currentSteeringAngle)\n" +
                "{\n" +
                "this->currentSteeringAngle = currentSteeringAngle;\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "steeringAngle = currentSteeringAngle;\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol, mathSymbol));

    }

    @Test
    public void testBasicGenericInstance() throws Exception {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicGenericInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        /*System.out.println(componentSymbol.getSubComponents().iterator().next().toString());
        for(ResolutionDeclarationSymbol sym:componentSymbol.getSubComponents().iterator().next().getResolutionDeclarationSymbols())
        {
            System.out.println(sym.getNameToResolve());
        }*/
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testBasicGenericInstance");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testBasicGenericArrayInstance() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicGenericArrayInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testBasicGenericArrayInstance");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testBasicGenericArrayInstance/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixModifierInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.matrixModifier", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testMathUnitInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testObjectDetectorInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("detection.objectDetector", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetector");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testParameterInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.lookUpInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testLookUpInstance");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    @Test
    public void testDoubleAccess() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.doubleAccess", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testDoubleAccess");
        generatorCPP.generateFiles(componentSymbol, symtab);
    }

    //@Ignore
    @Test
    public void testSimulatorMainController() throws IOException {
        ConstantPortSymbol.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("simulator.mainController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/simulatorMainController");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "simulatorMainController/";
        testFilesAreEqual(files, restPath);
    }

    //@Ignore
    @Test
    public void testBasicPrecision1() throws IOException {
        ConstantPortSymbol.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.basicPrecisionTest1", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/test");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "test/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMathUnitBothOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("paper.mathUnit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l3");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "paperMatrixModifier/l3/";
        testFilesAreEqual(files, restPath);
    }

    //TODO find out what is causing travis to fail this test
    @Ignore
    @Test
    public void testForLoopIf() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("testing.forLoopIfInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testing/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    @Ignore
    public void testMyComponent2() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        ExpandedComponentInstanceSymbol componentSymbol = symTab.<ExpandedComponentInstanceSymbol>resolve(
                "testing.subpackage2.myComponent2",
                ExpandedComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symTab);
        Assert.assertEquals(2, files.size());
    }
}
