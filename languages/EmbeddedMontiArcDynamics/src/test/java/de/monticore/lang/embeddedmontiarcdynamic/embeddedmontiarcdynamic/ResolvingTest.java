/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAConnectorInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.EmbeddedMontiArcDynamicLanguage;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.cncModel.EMADynamicComponentSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.embeddedmontiarcdynamic._symboltable.instanceStructure.EMADynamicComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

public class ResolvingTest extends AbstractTest{

    @Test
    public void ResolvingTest_01_Parser() throws IOException {

        test("./src/test/resources/test/embeddedmontiarcdynamic/parser", "./src/test/resources/test/embeddedmontiarcdynamic");
    }

    @Test
    public void ResolvingTest_02_ConnectorPort() throws IOException {

        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
//        Optional<EMADynamicComponentSymbol> comp = symScope.<EMADynamicComponentSymbol>resolve("easy.NotAdapter", EMADynamicComponentSymbol.KIND);
        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("easy.notAdapter", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());

        System.out.println(comp.get());

        for (EMAConnectorInstanceSymbol connector : comp.get().getConnectorInstances()) {
            System.out.println(connector);
            System.out.println("SourcePort: "+connector.getSourcePort());
            System.out.println("TargetPort: "+connector.getTargetPort());
        }

    }


    @Test
    public void ResolvingTest_03_Paper() throws IOException {

        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("paper.MathUnit", EMADynamicComponentSymbol.KIND);
        assertNotNull(comp1);

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("paper.mathUnit", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());

        System.out.println(comp.get());

        for (EMAConnectorInstanceSymbol connector : comp.get().getConnectorInstances()) {
            System.out.println(connector);
            System.out.println("SourcePort: "+connector.getSourcePort());
            System.out.println("TargetPort: "+connector.getTargetPort());
        }

    }

    @Test
    public void ResolvingTest_04_Syb() throws IOException {

        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("sub.Not", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("sub.outer", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());

        System.out.println(comp.get());

        List<EMAComponentInstanceSymbol> list = comp.get().getIndependentSubComponents();
        System.out.println(list);

        for (EMAConnectorInstanceSymbol connector : comp.get().getConnectorInstances()) {
            System.out.println(connector);
            System.out.println("SourcePort: "+connector.getSourcePort());
            System.out.println("TargetPort: "+connector.getTargetPort());
        }

    }

    @Test
    public void ResolvingTest_Event_00_TrueTest(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/event/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("test00.TrueTest", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("test00.trueTest", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());

        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_Event_01_TrueTest(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/event/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("test01.TrueTest", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("test01.trueTest", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());

        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_Infinite_Test1(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("infinite.InfTest1", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("infinite.infTest1", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
    }

    @Test
    public void ResolvingTest_Infinite_Test2(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("infinite.InfTest2", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("infinite.infTest2", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
    }

    @Test
    public void ResolvingTest_Fancy_01_Test(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("instanceFancy.Test1", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());
        System.out.println(comp1.get());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("instanceFancy.test1", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_Fancy_02_Test(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("instanceFancy.Test2", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());
        System.out.println(comp1.get());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("instanceFancy.test2", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_FromInner_01_Test(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("connectionFromInner.Test1", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());
        System.out.println(comp1.get());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("connectionFromInner.test1", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_Sub_01_Test(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("sub.SubSubSub", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());
        System.out.println(comp1.get());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("sub.subSubSub", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
        System.out.println("------------------------------------------------------------------------------------");
        System.out.println(comp1.get());
        System.out.println("------------------------------------------------------------------------------------");
        System.out.println(comp.get());
    }

    @Test
    public void ResolvingTest_Sub_01_TypeArgs(){
        Scope symScope = createSymTab("./src/test/resources/test/embeddedmontiarcdynamic/");
        Optional<EMADynamicComponentSymbol> comp1 = symScope.<EMADynamicComponentSymbol>resolve("sub.TypeArgs", EMADynamicComponentSymbol.KIND);
        assertTrue(comp1.isPresent());
        System.out.println(comp1.get());

        Optional<EMADynamicComponentInstanceSymbol> comp = symScope.<EMADynamicComponentInstanceSymbol>resolve("sub.typeArgs", EMADynamicComponentInstanceSymbol.KIND);

        assertTrue(comp.isPresent());
        System.out.println("------------------------------------------------------------------------------------");
        System.out.println(comp1.get());
        System.out.println("------------------------------------------------------------------------------------");
        System.out.println(comp.get());
    }

    private void test(String path, String base) throws IOException {
        Scope symScope = createSymTab(base);

        ResolveTest rTest = new ResolveTest("." + EmbeddedMontiArcDynamicLanguage.FILE_ENDING, base, symScope);
        Files.walkFileTree(Paths.get(path), rTest);
        if (!rTest.getErrors().isEmpty()) {
            Log.debug("Models in error", "ResolverTest");
            for (String model : rTest.getErrors()) {
                Log.debug("  " + model, "Resolver");
            }
        }
        Log.info(
                "Count of tested models: " + rTest.getTestCount(),
                "Resolver"
        );
        Log.info(
                "Count of correctly resolved models: "
                        + (rTest.getTestCount() - rTest.getErrors().size()),
                "ParserTest"
        );

        assertTrue(
                "There were models that could not be resolved",
                rTest.getErrors().isEmpty()
        );
    }


    private static class ResolveTest extends SimpleFileVisitor<Path> {

        private String fileEnding;
        private String basePath;
        private Scope symTab;
        private List<String> errors = new ArrayList<>();
        private int testCount = 0;

        public ResolveTest(String fileEnding, String basePath, Scope symTab) {
            super();
            this.fileEnding = fileEnding;
            this.basePath = basePath;
            this.symTab = symTab;
        }

        /**
         * @return testCount
         */
        public int getTestCount() {
            return this.testCount;
        }

        /**
         * @return modelsInError
         */
        public List<String> getErrors() {
            return this.errors;
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
            Path par = file.getParent();
            if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
                Path rel = Paths.get(basePath).relativize(file.getParent());

                String pack = rel.toString().replace(File.separator, ".");
                String name = file.toFile().getName();
                name = name.substring(0, name.lastIndexOf('.'));

                String eventFullName = pack+"."+name;

                Log.debug("Resolving dynamic component  " + eventFullName, "ResolveTest");
                testCount++;

                Optional<EMADynamicComponentSymbol> comp = Optional.empty();
                try {
                    comp = symTab.<EMADynamicComponentSymbol>resolve(
                            eventFullName, EMADynamicComponentSymbol.KIND);
                }catch (Exception ex){
                    ex.printStackTrace();
                }

                if(!comp.isPresent()){
                    errors.add(file.toString());
                    Log.error("There were unexpected parser errors");
                }else{
                    Log.getFindings().clear();
                }
            }
            return FileVisitResult.CONTINUE;
        }


    }
}
