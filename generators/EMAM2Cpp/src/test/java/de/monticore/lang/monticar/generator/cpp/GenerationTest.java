/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.helper.ConstantPortHelper;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.monticar.generator.Helper;
import de.monticore.lang.monticar.generator.optimization.ThreadingOptimizer;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

/**
 */
public class GenerationTest extends AbstractSymtabTest {

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Test
    public void testBooleanVariableComp() throws IOException{
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.booleanVariableComp", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testBooleanVariableComp");
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "testBooleanVariableComp/";
        testFilesAreEqual(files, restPath);
    }
  
    @Test
    public void testSingleElemArray() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.singleElemArray", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testSingleElemArray");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "testSingleElemArray/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testBasicConstantAssignment() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicConstantAssignment", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testConstantAssignment");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "testConstantAssignment/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testBasicConstantAssignment2() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicConstantAssignment2", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testConstantAssignment2");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);
        String restPath = "testConstantAssignment2/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testBasicPorts() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicPorts", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

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
                "out2 = in1;\n" +
                "out1 = in2;\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol));

    }

    @Test
    public void testBasicPortsConstantConnector() {
//        ConstantPortSymbol.resetLastID();
        ConstantPortHelper.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicPortsConstantConnector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        //System.out.println(generatorCPP.generateString(componentSymbol));
        assertEquals("#ifndef TEST_BASICPORTSCONSTANTCONNECTOR\n" +
                "#define TEST_BASICPORTSCONSTANTCONNECTOR\n" +
                "#ifndef M_PI\n" +
                "#define M_PI 3.14159265358979323846\n" +
                "#endif\n" +
                "#include \"octave/oct.h\"\n" +
                "class test_basicPortsConstantConnector{\n" +
                "public:\n" +
                "bool CONSTANTPORT1;\n" +
                "double CONSTANTPORT2;\n" +
                "double out1;\n" +
                "bool out2;\n" +
                "void init()\n" +
                "{\n" +
                "this->CONSTANTPORT1 = true;\n" +
                "this->CONSTANTPORT2 = 1;\n" +
                "}\n" +
                "void execute()\n" +
                "{\n" +
                "out1 = 1;\n" +
                "out2 = true;\n" +
                "}\n" +
                "\n" +
                "};\n" +
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol));

    }


    @Test
    public void testPortsMath() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicPortsMath", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        //MathStatementsSymbol mathSymbol = symtab.<MathStatementsSymbol>resolve("test.BasicPortsMath.MathStatements", MathStatementsSymbol.KIND).orElse(null);
        //assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

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
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol));

    }

    @Test
    public void testPortsLoop() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicPortsLoop", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        assertNotNull(mathSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

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
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol));

    }

    @Ignore
    @Test
    public void testSimulatorSpeedLimitChecker() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulator.speedLimitChecker", EMAComponentInstanceSymbol.KIND).orElse(null);
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
                "#endif\n", generatorCPP.generateString(symtab, componentSymbol));

    }

    @Ignore
    @Test
    public void testSimulatorBrakeController() {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulator.BrakeController", EMAComponentInstanceSymbol.KIND).orElse(null);
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

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("steerController", EMAComponentInstanceSymbol.KIND).orElse(null);
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

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicGenericInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        /*System.out.println(componentSymbol.getSubComponents().iterator().next().toString());
        for(ResolutionDeclarationSymbol sym:componentSymbol.getSubComponents().iterator().next().getResolutionDeclarationSymbols())
        {
            System.out.println(sym.getNameToResolve());
        }*/
        MathStatementsSymbol mathSymbol = Helper.getMathStatementsSymbolFor(componentSymbol, symtab);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testBasicGenericInstance");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testBasicGenericArrayInstance() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicGenericArrayInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testBasicGenericArrayInstance");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testBasicGenericArrayInstance/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixModifierInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.matrixModifier", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l0");
        generatorCPP.generateFiles(symtab, componentSymbol);;

        generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testMathUnitInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit4", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l0");
        generatorCPP.generateFiles(symtab, componentSymbol);;
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l1");
        generatorCPP.generateFiles(symtab, componentSymbol);;
        generatorCPP.setUseAlgebraicOptimizations(false);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l2");
        generatorCPP.generateFiles(symtab, componentSymbol);;
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/MatrixModifier/l3");
        generatorCPP.generateFiles(symtab, componentSymbol);;

    }

    @Test
    public void testObjectDetectorInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("detection.objectDetector", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/detectionObjectDetector");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testParameterInstancing() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.lookUpInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testLookUpInstance");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    @Test
    public void testDoubleAccess() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.doubleAccess", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testDoubleAccess");
        generatorCPP.generateFiles(symtab, componentSymbol);;
    }

    //@Ignore
    @Test
    public void testSimulatorMainController() throws IOException {
        //TODO: new testfile
//        ConstantPortSymbol.resetLastID();
        ConstantPortHelper.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("simulator.mainController", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/simulatorMainController");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "simulatorMainController/";
        testFilesAreEqual(files, restPath);
    }

    //@Ignore
    @Test
    public void testBasicPrecision1() throws IOException {
//        ConstantPortSymbol.resetLastID();
        ConstantPortHelper.resetLastID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("test.basicPrecisionTest1", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/test");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "test/";
        testFilesAreEqual(files, restPath);
    }

    @Ignore
    @Test
    public void testMathUnitBothOptimizations() throws IOException {
        ThreadingOptimizer.resetID();
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentSymbol component = symtab.<EMAComponentSymbol>resolve("paper.MathUnit", EMAComponentSymbol.KIND).orElse(null);
        assertNotNull(component);
        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("paper.mathUnit", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setUseAlgebraicOptimizations(true);
        generatorCPP.setUseThreadingOptimization(true);

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/paperMatrixModifier/l3");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "paperMatrixModifier/l3/";
        testFilesAreEqual(files, restPath);
    }

    //TODO find out what is causing travis to fail this test
    @Ignore
    @Test
    public void testForLoopIf() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        EMAComponentInstanceSymbol componentSymbol = symtab.<EMAComponentInstanceSymbol>resolve("testing.forLoopIfInstance", EMAComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing");
        List<File> files = generatorCPP.generateFiles(symtab, componentSymbol);;
        String restPath = "testing/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    @Ignore // No recursive struct references coco !
    public void testMyComponent2() throws IOException {
        cppCodeForMyComponentXCanBeGenerated(2);
    }

    @Test
    public void cppCodeForMyComponent3CanBeGenerated() throws IOException {
        cppCodeForMyComponentXCanBeGenerated(3);
    }

    @Test
    public void cppCodeForMyComponent4v1CanBeGenerated() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subpackage4.myComponent4v1",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/MyComponent4v1");
        List<File> files = generatorCPP.generateFiles(symTab, componentSymbol);;
        Assert.assertNotNull(files);
        Assert.assertFalse(files.isEmpty());
    }

    @Test
    public void testDemuxTest() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.demuxTest",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing");
        List<File> files = generatorCPP.generateFiles(symTab, componentSymbol);;
        String restPath = "testing/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    @Ignore("https://github.com/EmbeddedMontiArc/EMAM2Cpp/issues/14")
    public void cppCodeForMyComponent4v2CanBeGenerated() throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                "testing.subpackage4.myComponent4v2",
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/MyComponent4v2");
        List<File> files = generatorCPP.generateFiles(symTab, componentSymbol);;
        Assert.assertNotNull(files);
        Assert.assertFalse(files.isEmpty());
    }

    @Test
    public void cppCodeForMyComponent5CanBeGenerated() throws IOException {
        cppCodeForMyComponentXCanBeGenerated(5);
    }

    @Test
    @Ignore("https://github.com/EmbeddedMontiArc/EMAM2Cpp/issues/27")
    public void cppCodeForMyComponent6CanBeGenerated() throws IOException {
        cppCodeForMyComponentXCanBeGenerated(6);
    }

    @Test
    public void cppCodeForMyComponent7CanBeGenerated() throws IOException {
        cppCodeForMyComponentXCanBeGenerated(7);
    }

    private void cppCodeForMyComponentXCanBeGenerated(int x) throws IOException {
        TaggingResolver symTab = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentSymbol = symTab.<EMAComponentInstanceSymbol>resolve(
                String.format("testing.subpackage%1$s.myComponent%1$s", x),
                EMAComponentInstanceSymbol.KIND
        ).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath(
                String.format("./target/generated-sources-cpp/testing/MyComponent%s", x)
        );
        List<File> files = generatorCPP.generateFiles(symTab, componentSymbol);;
        Assert.assertNotNull(files);
        Assert.assertFalse(files.isEmpty());
    }

    @Test
    public void testParameterInstance() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("testing.parameterInstance",EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();

        generatorCPP.useOctaveBackend();
        generatorCPP.setGenerateCMake(false);

        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/Parameter/");

        List<File> files = generatorCPP.generateFiles(taggingResolver, componentInstanceSymbol);

        testFilesAreEqual(files,"testing/Parameter/");

    }

    @Test
    public void testWholeNumberPort() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("testing.wholeNumberPort",EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/WholeNumberPort/");

        List<File> files = generatorCPP.generateFiles(taggingResolver, componentInstanceSymbol);

        testFilesAreEqual(files,"testing/WholeNumberPort/");
    }

    @Test
    public void testWholeNumberMatrix() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("testing.wholeNumberMatrix",EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testing/WholeNumberMatrix/");

        List<File> files = generatorCPP.generateFiles(taggingResolver, componentInstanceSymbol);

        testFilesAreEqual(files,"testing/WholeNumberMatrix/");
    }

    @Test
    public void testStructDecl() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources");
        EMAComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<EMAComponentInstanceSymbol>resolve("structs.structDeclarationComp",EMAComponentInstanceSymbol.KIND).orElse(null);

        assertNotNull(componentInstanceSymbol);

        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerateCMake(false);
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/structs/StructDeclarationComp/");

        List<File> files = generatorCPP.generateFiles(taggingResolver, componentInstanceSymbol);

        //testFilesAreEqual(files,"testing/WholeNumberMatrix/");

    }
}
