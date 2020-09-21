/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarcdynamic.event;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.embeddedmontiarcdynamic.event._ast.ASTEventCompilationUnit;
import de.monticore.lang.embeddedmontiarcdynamic.event._parser.EventParser;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.ComponentEventSymbol;
import de.monticore.lang.embeddedmontiarcdynamic.event._symboltable.EventLanguage;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static junit.framework.TestCase.fail;
import static org.junit.Assert.assertTrue;

public class ResolvingTest extends AbstractTest{


    public static final boolean ENABLE_FAIL_QUICK = false;

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }


    @Test
    public void testResolveBoolean()  {

        try {
            test("./src/test/resources/test/event/boolean", "./src/test/resources/test/event");
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (Log.getErrorCount() > 0) {
            fail("Test Failed, found errors");
        }
    }

    @Test
    public void testResolvePortValue()  {

        try {
            test("./src/test/resources/test/event/portvalue", "./src/test/resources/test/event");
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (Log.getErrorCount() > 0) {
            fail("Test Failed, found errors");
        }
    }

    private void test(String path, String base) throws IOException {
        Scope symScope = createSymTab(base);

        ResolveTest rTest = new ResolveTest("." + EventLanguage.FILE_ENDING, base, symScope);
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

                Log.debug("Resolving event " + eventFullName, "ResolveTest");
                testCount++;

                Optional<ComponentEventSymbol> comp = Optional.empty();
                try {
                    comp = symTab.<ComponentEventSymbol>resolve(
                            eventFullName, ComponentEventSymbol.KIND);
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
/*
    @Test
    public void zwTest(){
        Scope symTab = createSymTab("./src/test/resources/test/event");

        Optional<ComponentEventSymbol> comp = Optional.empty();
        try {
            comp = symTab.<ComponentEventSymbol>resolve(
                    "parser.Test2", ComponentEventSymbol.KIND);
            System.out.println(comp.toString());
        }catch (Exception ex){
            ex.printStackTrace();
        }

    }*/

}
