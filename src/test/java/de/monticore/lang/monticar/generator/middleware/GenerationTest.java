package de.monticore.lang.monticar.generator.middleware;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos.EmbeddedMontiArcMathCoCos;
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
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
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
    public void testCppOnlyMiddlewareGeneration() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.a.addComp", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/CMakeCppOnly/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(),"cpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        testFilesAreEqual(files,"CMakeCppOnly/src/",generationTargetPath);

    }

    @Ignore("Planner no longer used")
    @Test
    public void plannerTest() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.vehicle.planner", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        ComponentSymbol componentSymbol = taggingResolver.<ComponentSymbol>resolve("ba.vehicle.Planner",ComponentSymbol.KIND).orElse(null);

        EmbeddedMontiArcMathCoCos.createChecker().checkAll((ASTComponent)componentSymbol.getAstNode().orElse(null));

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/Planner/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(),"cpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        //known errors:
        //wayOut(1-1, i-1)
        //wayout[1-1,i-1]
        //Helper::getDoubleFromOctaveListFirstResult(Fasin(Helper::convertToOctaveValueList(deltaY/dist),1)
        //=> Fasin(deltaY/dist) => std::sin(...)
        //...
        fixKnownErrors(files);


        testFilesAreEqual(files,"Planner/src/",generationTargetPath);

    }

    @Test
    public void testBaSystem() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.system", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        ComponentSymbol componentSymbol = taggingResolver.<ComponentSymbol>resolve("ba.System",ComponentSymbol.KIND).orElse(null);

        EmbeddedMontiArcMathCoCos.createChecker().checkAll((ASTComponent)componentSymbol.getAstNode().orElse(null));

        componentInstanceSymbol.getPorts().forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));
        componentInstanceSymbol.getSubComponents().stream()
                .flatMap(sc -> sc.getPorts().stream())
                .forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));

        List<ConnectorSymbol> stopCommConnectors = componentInstanceSymbol.getConnectors()
                .stream()
                .filter(con -> con.getTarget().startsWith("stopCommQuality") || con.getSource().startsWith("stopCommQuality"))
                .sorted(Comparator.comparing(ConnectorSymbol::getName))
                .collect(Collectors.toList());

        for(ConnectorSymbol con : stopCommConnectors){
            String indexString = con.getName().replaceAll(".*\\[(\\d+)\\].*","$1");
            if(con.getSource().startsWith("stopCommQuality")){
                con.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol("/v"+ indexString + "/comm/in/slowDown"+ indexString,"std_msgs/Bool","data"));
                con.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol("/v"+ indexString + "/comm/in/slowDown"+ indexString,"std_msgs/Bool","data"));

            }else{
                con.getSourcePort().setMiddlewareSymbol(new RosConnectionSymbol("/sim/comm/slowDown" + indexString,"std_msgs/Bool","data"));
                con.getTargetPort().setMiddlewareSymbol(new RosConnectionSymbol("/sim/comm/slowDown" + indexString,"std_msgs/Bool","data"));

            }
        }

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/system/src/";
        distributedTargetGenerator.setGenerationTargetPath(generationTargetPath);
        distributedTargetGenerator.setGenDebug(true);
        distributedTargetGenerator.add(new CPPGenImpl(),"cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(),"roscpp");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);
        fixKnownErrors(files);
    }

    @Test
    public void testSetCompareInstance() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.tests.setCompareInstance", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/setCompareInstance/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(),"cpp");
        //middlewareGenerator.add(new RosCppGenImpl(),"roscpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        fixKnownErrors(files);
    }

    @Test
    public void testLaneIntersection() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.util.lineIntersection", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/laneIntersection/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(),"cpp");
        //middlewareGenerator.add(new RosCppGenImpl(),"roscpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        fixKnownErrors(files);
    }

    @Test
    public void testRectIntersection() throws IOException{
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("ba.util.rectIntersection", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        //EmbeddedMontiArcMathCoCos.createChecker().checkAll((ASTComponent)componentInstanceSymbol.getComponentType().getReferencedSymbol().getAstNode().orElse(null));

        MiddlewareGenerator middlewareGenerator = new MiddlewareGenerator();
        String generationTargetPath = "./target/generated-sources-cmake/rectIntersection/src/";
        middlewareGenerator.setGenerationTargetPath(generationTargetPath);
        middlewareGenerator.add(new CPPGenImpl(),"cpp");
        //middlewareGenerator.add(new RosCppGenImpl(),"roscpp");

        List<File> files = middlewareGenerator.generate(componentInstanceSymbol, taggingResolver);
        fixKnownErrors(files);
    }

    private void fixKnownErrors(List<File> files) throws IOException {
        //known errors:
        //wayOut(1-1, i-1)
        //=> wayOut[1-1,i-1]
        //Helper::getDoubleFromOctaveListFirstResult(Fasin(Helper::convertToOctaveValueList(deltaY/dist),1)
        //=> Fasin(deltaY/dist) => std::sin(...)
        //...
        for(File f : files) {
            Path path = Paths.get(f.getAbsolutePath());
            Charset charset = StandardCharsets.UTF_8;
            String content = new String(Files.readAllBytes(path), charset);
            content = content.replace("#include \"octave/builtin-defun-decls.h\"","#include <cmath>");
            content = content.replaceAll("\\(Helper::getDoubleFromOctaveListFirstResult\\(([\\w|/]*)\\(Helper::convertToOctaveValueList\\(([\\w|/|\\(|\\)|,| |-]*)\\),1\\)\\)\\)","$1($2)");
            content = content.replaceAll("(\\w+)\\(([\\w]+\\-1)\\)","$1[$2]");
            content = content.replace("LaneletPaircurLaneletPair", "ba_util_LaneletPair curLaneletPair");
            content = content.replace("Fasin","std::asin");
            content = content.replace("Fsin","std::sin");
            content = content.replace("Fcos","std::cos");
            content = content.replace("Fabs","std::abs");

            content = content.replace("Col<int> counter=Col<int>(1);" , "Col<int> counter=Col<int>(n);");
            content = content.replace("colvec tmpLine;","colvec tmpLine = colvec(4);");
//            content = content.replace("sqrt","std::sqrt");

            if(f.getName().equals("ba_system_collisionDetection_multiOr.h")){
                content = content.replace("10","1");
            }

//            if(f.getName().equals("ba_system_collisionDetection_rectIntersection_1__dualSetCompare.h")){
//                String fixedGenerics = "const int n = 4;\nconst int n2 = 10;\n";
//                content = content.replace("const int cols = 1;","const int cols = 1;\n" + fixedGenerics);
//            }

            if(f.getName().equals("ba_system_collisionDetection_rectIntersection_1__dualSetCompare.h")){
                content = content.replace("rows = 4","rows = 2");
                content = content.replace("cols = 1","cols = 4");
//                content = content.replace("n = 4","n = 2");
//                content = content.replace("n2 = 10","n2 = 1");
            }


            Files.write(path, content.getBytes(charset));
        }
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

    @Ignore("Part of system now. See testBaSystem")
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



        componentInstanceSymbol.getPorts().forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));
        componentInstanceSymbol.getSubComponents().stream()
        .flatMap(subc -> subc.getConnectors().stream())
                .filter(c -> c.getSourcePort().equals(c.getTargetPort()))
                .forEach(c -> System.out.println("Source = Target in comp "+c.getComponentInstance().get().getName()+":"+c.getSource() + " -> " + c.getTargetPort()));

        componentInstanceSymbol.getSubComponents().stream()
                .flatMap(subc -> subc.getPorts().stream())
                .forEach(p -> p.setMiddlewareSymbol(new RosConnectionSymbol()));


        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/intersection/src/");

        distributedTargetGenerator.add(new CPPGenImpl(), "cpp");
        distributedTargetGenerator.add(new RosCppGenImpl(), "roscpp");

        List<File> files = distributedTargetGenerator.generate(componentInstanceSymbol, taggingResolver);


        //Workaround for compiler errors: change
        //conflictIn(i-1) to conflictIn[i-1]
        //indexLookupIn(i-1) to indexLookupIn[i-1]
        for(File f : files) {
            Path path = Paths.get(f.getAbsolutePath());
            Charset charset = StandardCharsets.UTF_8;
            String content = new String(Files.readAllBytes(path), charset);
            content = content.replace("conflictIn(i-1)", "conflictIn[i-1]");
            content = content.replace("indexLookupIn(i-1)", "indexLookupIn[i-1]");
            content = content.replace("Col<int> counter=Col<int>(1);","Col<int> counter=Col<int>(2);");
            Files.write(path, content.getBytes(charset));
        }
    }

    @Test
    public void testParameterInit() throws IOException {
        TaggingResolver taggingResolver = createSymTabAndTaggingResolver("src/test/resources/");
        RosToEmamTagSchema.registerTagTypes(taggingResolver);

        ExpandedComponentInstanceSymbol componentInstanceSymbol = taggingResolver.<ExpandedComponentInstanceSymbol>resolve("tests.dist.parameterInit", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentInstanceSymbol);

        DistributedTargetGenerator distributedTargetGenerator = new DistributedTargetGenerator();
        distributedTargetGenerator.setGenerationTargetPath("./target/generated-sources-cmake/paramInit/src/");
        distributedTargetGenerator.add(new CPPGenImpl(),"cpp");
        distributedTargetGenerator.generate(componentInstanceSymbol,taggingResolver);
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
