/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct;

import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
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

import static org.junit.Assert.assertTrue;

public class ParserTest {
    public static final boolean ENABLE_FAIL_QUICK = false;

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }


    @Test
    public void testStruct() throws Exception {
        test("struct");
        if (Log.getErrorCount() > 0) {
            throw new Exception("Test Failed, found errors");
        }
    }

    private void test(String fileEnding) throws IOException {
        ParseTest parserTest = new ParseTest("." + StructLanguage.FILE_ENDING);
        Files.walkFileTree(Paths.get("src/test/resources/test/parser"), parserTest);
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

    /**
     * Visits files of the given file ending and checks whether they are parsable.
     *
     * @see Files#walkFileTree(Path, java.nio.file.FileVisitor)
     */
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
            for(Path p:file) {
                java.io.File tmp1 = p.toFile();
                String tmp2 = p.toString();
                int i = 0;
            }
            if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
                Log.debug("Parsing file " + file.toString(), "ParserTest");
                testCount++;
                StructParser parser = new StructParser();
                Optional<ASTStructCompilationUnit> parsedStruct = parser.parse(file.toString());
                if (parser.hasErrors() || !parsedStruct.isPresent()) {
                    errors.add(file.toString());
                    Log.error("There were unexpected parser errors");
                } else {
                    Log.getFindings().clear();
                }
            }
            return FileVisitResult.CONTINUE;
        }
    }
}
