package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.middleware.ros.RosToEmamTagSchema;
import de.monticore.lang.monticar.emadl.generator.EMADLAbstractSymtab;
import de.monticore.lang.monticar.generator.middleware.impls.*;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.se_rwth.commons.logging.Log;
import freemarker.template.TemplateException;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;

public class GenerationTest extends AbstractSymtabTest {

    public static final String TEST_PATH = "src/test/resources/";

    @Test
    public void testBasicGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new StarBridgeGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-emam/basicGeneration/src/");
        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testCMakeGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new CMakeGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-cmake/CMakeGeneration/src/");
        starBridgeGenerator.add(new CPPGenImpl(), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testCppOnlyMiddlewareGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/CMakeCppOnly/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(), "cpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        testFilesAreEqual(files, "CMakeCppOnly/src/", generationTargetPath);

    }

    @Test
    public void testBaSystem() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.system", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        ComponentSymbol componentSymbol = taggingResolver.<ComponentSymbol>resolve("ba.System", ComponentSymbol.KIND).orElse(null);

        EmbeddedMontiArcMathCoCos.createChecker().checkAll((ASTComponent) componentSymbol.getAstNode().orElse(null));

        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/system/src/";
        distributedTargetGenerator.setGenerationTargetPath(generationTargetPath);
        //distributedTargetGenerator.setGenDebug(true);
        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);

        testFilesAreEqual(files, "system/src/", generationTargetPath);
    }

    @Test
    public void testMiddlewareGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        //register the middleware tag types
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        //make sure the middleware tags are loaded
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        middlewareGenerator.setGenerationTargetPath("./target/generated-sources-cmake/middlewareGenerator/src/");
        //generator for component itself
        middlewareGenerator.add(new CPPGenImpl(), "cpp");
        //generator for the ros connection
        middlewareGenerator.add(new RosCppGenImpl(), "roscpp");
        //generator for a dummy to test support for multiple middleware at the same time
        middlewareGenerator.add(new DummyMiddlewareGenImpl(), "dummy");

        middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testDistributedTargetGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.distComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);


        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/distributed/src/");

        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testDistributedStructTargetGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.distWithStructComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);


        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/distributedStruct/src/");

        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testParameterInit() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.parameterInit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/paramInit/src/");
        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testMutliMwGenerateAll() throws IOException {
        testMutliMw("allMw", true, true);
    }

    @Test
    public void testMutliMwGenerateSome() throws IOException {
        testMutliMw("someMw", true, false);
    }

    @Test
    public void testMutliMwGenerateNone() throws IOException {
        testMutliMw("noneMw", false, false);
    }

    public void testMutliMw(String relPath, boolean genRosAdapter, boolean genDummyAdapter) throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        //Don't load tags, will be set manually
        //RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        PortSymbol in1 = componentInstanceSymbol.getPort("in1").orElse(null);
        assertNotNull(in1);

        PortSymbol in2 = componentInstanceSymbol.getPort("in2").orElse(null);
        assertNotNull(in2);

        PortSymbol out1 = componentInstanceSymbol.getPort("out1").orElse(null);
        assertNotNull(out1);

        if (genRosAdapter) {
            in1.setMiddlewareSymbol(new RosConnectionSymbol("/test", "std_msgs/Float64", "data"));
            in2.setMiddlewareSymbol(new RosConnectionSymbol("/test2", "std_msgs/Float64", "data"));
        }

        if (genDummyAdapter) {
            out1.setMiddlewareSymbol(new DummyMiddlewareSymbol());
        }

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/" + relPath + "/src/");
        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");
        distributedTargetGenerator.add(new DummyMiddlewareGenImpl(), "dummy");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);

        List<String> fileNames = files.stream()
                .map(File::getName)
                .collect(Collectors.toList());

        assertEquals(fileNames.contains("RosAdapter_tests_a_addComp.h"), genRosAdapter);
        assertEquals(fileNames.contains("DummyAdapter_tests_a_addComp.h"), genDummyAdapter);
    }

    @Ignore
    //Cpp:  No 3 dim matrices(https://github.com/EmbeddedMontiArc/EMAM2Cpp/issues/37)
    //      No types other then Q(https://github.com/EmbeddedMontiArc/EMAM2Cpp/issues/14)
    @Test
    public void testThreeDimMatrix() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        //register the middleware tag types
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.matrix.threeDimMatrixComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        //make sure the middleware tags are loaded
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        middlewareGenerator.setGenerationTargetPath("./target/generated-sources-cmake/threeDimMatrixComp/src/");
        //generator for component itself
        middlewareGenerator.add(new CPPGenImpl(), "cpp");
        //generator for the ros connection
        middlewareGenerator.add(new RosCppGenImpl(), "roscpp");

        middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testTwoDimMatrix() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        //register the middleware tag types
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.matrix.twoDimMatrixComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        //make sure the middleware tags are loaded
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        middlewareGenerator.setGenerationTargetPath("./target/generated-sources-cmake/twoDimMatrixComp/src/");
        //generator for component itself
        middlewareGenerator.add(new CPPGenImpl(), "cpp");
        //generator for the ros connection
        middlewareGenerator.add(new RosCppGenImpl(), "roscpp");

        middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
    }

    @Test
    public void testEMADLMiddlewareGeneration() throws IOException, TemplateException {
        TaggingResolver taggingResolver = EMADLAbstractSymtab.createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.emadlTests.resNet34", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        middlewareGenerator.setGenerationTargetPath("./target/generated-sources-emadl/");
        middlewareGenerator.add(new EMADLGeneratorImpl(TEST_PATH, "MXNET"), "cpp");
        middlewareGenerator.add(new RosCppGenImpl(), "roscpp");

        middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        assertTrue(Log.getFindings().isEmpty());
    }

    @Test
    public void testBasicEMADLGeneration() throws IOException {
        TaggingResolver taggingResolver = EMADLAbstractSymtab.createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.aEmadl.compA", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        StarBridgeGenerator starBridgeGenerator = new StarBridgeGenerator();
        starBridgeGenerator.setGenerationTargetPath("./target/generated-sources-emadl-ros/");
        starBridgeGenerator.add(new EMADLGeneratorImpl(TEST_PATH, "MXNET"), "cpp");
        starBridgeGenerator.add(new RosCppGenImpl(), "roscpp");

        starBridgeGenerator.generate(componentInstanceSymbol, taggingResolver);
        assertTrue(Log.getFindings().isEmpty());
    }


    @Test
    public void testLabComp() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver(TEST_PATH);
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("lab.system", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        //make sure the middleware tags are loaded
        Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);


        DistributedTargetGenerator middlewareGenerator = new DistributedTargetGenerator();
        middlewareGenerator.setGenerationTargetPath("./target/generated-sources-cmake/lab/src/");
        //generator for component itself
        middlewareGenerator.add(new CPPGenImpl(), "cpp");
        //generator for the ros connection
        middlewareGenerator.add(new RosCppGenImpl(), "roscpp");

        middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);


    }
}
