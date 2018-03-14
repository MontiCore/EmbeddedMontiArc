package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosConnectionSymbol;
import de.monticore.lang.embeddedmontiarc.tagging.RosToEmamTagSchema;
import de.monticore.lang.monticar.generator.cpp.GeneratorCPP;
import de.monticore.lang.monticar.generator.cpp.TestConverter;
import de.monticore.lang.monticar.generator.middleware.impls.CPPGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.DummyMiddlewareGenImpl;
import de.monticore.lang.monticar.generator.middleware.impls.DummyMiddlewareSymbol;
import de.monticore.lang.monticar.generator.middleware.impls.RosCppGenImpl;
import de.monticore.lang.monticar.generator.roscpp.helper.TagHelper;
import de.monticore.lang.monticar.streamunits._symboltable.ComponentStreamUnitsSymbol;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

public class GenerationTest extends AbstractSymtabTest {

    @Test
    public void testBasicGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
    public void testMiddlewareGenerator() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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

    @Ignore
    //Workaround for compiler errors: change
    //conflictIn(i-1) to conflictIn[i-1]
    //indexLookupIn(i-1) to indexLookupIn[i-1]
    @Test
    public void testIntersectionGeneration() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.intersection.intersectionController", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);
        //Map<PortSymbol, RosConnectionSymbol> tags = TagHelper.resolveTags(taggingResolver, componentInstanceSymbol);

        componentInstanceSymbol.getConnectors().stream()
                .filter(c -> c.getSourcePort().equals(c.getTargetPort()))
                .forEach(c -> System.out.println("Source = Target:"+c.getSource() + " -> " + c.getTargetPort()));

        componentInstanceSymbol.getSubComponents().stream()
        .flatMap(subc -> subc.getConnectors().stream())
                .filter(c -> c.getSourcePort().equals(c.getTargetPort()))
                .forEach(c -> System.out.println("Source = Target in comp "+c.getComponentInstance().get().getName()+":"+c.getSource() + " -> " + c.getTargetPort()));

        componentInstanceSymbol.getPorts().forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));
        componentInstanceSymbol.getSubComponents().stream()
                .flatMap(subc -> subc.getPorts().stream())
                .forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));


        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/intersection/src/");

        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);

    }

    @Ignore
    @Test
    public void testsStreamTest() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.tests.relToAbsTrajectoryInst", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);


        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-emam/intersection/test/");
        generatorCPP.setModelsDirPath(Paths.get("src/test/resources"));
        generatorCPP.setGenerateTests(false);
//        generatorCPP.useArmadilloBackend();
        ComponentStreamUnitsSymbol streamSymbol = taggingResolver.<ComponentStreamUnitsSymbol>resolve("ba.tests.RelToAbsTrajectory", ComponentStreamUnitsSymbol.KIND).orElse(null);
        assertNotNull(streamSymbol);
        generatorCPP.generateFiles(componentInstanceSymbol,taggingResolver);
        generatorCPP.generateFile(TestConverter.generateMainTestFile(streamSymbol,componentInstanceSymbol));
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
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
}
