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

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static junit.framework.TestCase.assertNotNull;
import static junit.framework.TestCase.fail;
import static org.junit.Assert.assertTrue;

public class ParserTest extends AbstractTest{



    public static final boolean ENABLE_FAIL_QUICK = false;

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testParserBoolean() {

        try {
            test("src/test/resources/test/event/boolean");
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (Log.getErrorCount() > 0) {
             fail("Test Failed, found errors");
        }
    }

    @Test
    public void testParserPortValue()  {

        try {
            test("src/test/resources/test/event/portvalue");
        } catch (IOException e) {
            e.printStackTrace();
        }
        if (Log.getErrorCount() > 0) {
            fail("Test Failed, found errors");
        }
    }

    private void test(String path) throws IOException {
        ParseTest parserTest = new ParseTest("." + EventLanguage.FILE_ENDING);
        Files.walkFileTree(Paths.get(path), parserTest);
        if (!parserTest.getErrors().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getErrors()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info(
                "Count of tested models: " + parserTest.getTestCount(),
                "ParserTest"
        );
        Log.info(
                "Count of correctly parsed models: "
                        + (parserTest.getTestCount() - parserTest.getErrors().size()),
                "ParserTest"
        );
        assertTrue(
                "There were models that could not be parsed",
                parserTest.getErrors().isEmpty()
        );
    }

    private static class ParseTest extends SimpleFileVisitor<Path> {

        private String fileEnding;
        private List<String> errors = new ArrayList<>();
        private int testCount = 0;

        public ParseTest(String fileEnding) {
            super();
            this.fileEnding = fileEnding;
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
                Log.debug("Parsing file " + file.toString(), "ParserTest");
                testCount++;
                EventParser ep = new EventParser();
                Optional<ASTEventCompilationUnit> parsedEvent = ep.parse(file.toString());
                if(ep.hasErrors() || !parsedEvent.isPresent()){
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
