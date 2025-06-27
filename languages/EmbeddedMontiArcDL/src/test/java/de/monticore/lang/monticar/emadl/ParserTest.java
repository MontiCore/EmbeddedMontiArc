/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTEMACompilationUnit;
import de.monticore.lang.monticar.emadl._parser.EMADLParser;
import de.se_rwth.commons.logging.Log;
import org.junit.Before;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static junit.framework.TestCase.assertTrue;


public class ParserTest {

    public static final boolean ENABLE_FAIL_QUICK = false;
    private static List<String> expectedParseErrorModels = new ArrayList<String>()
            .stream().map(s -> Paths.get(s).toString())
            .collect(Collectors.toList());

    @Before
    public void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }


    @Test
    public void testEmbeddedMontiArcDL() throws Exception {
        test("emadl");
        if (Log.getErrorCount() > 0) {
            throw new Exception("Test Failed, found errors");
        }
    }

    private void test(String fileEnding) throws IOException {
        ParseTest parserTest = new ParseTest("." + fileEnding);
        Files.walkFileTree(Paths.get("src/test/resources/models"), parserTest);

        if (!parserTest.getModelsInError().isEmpty()) {
            Log.debug("Models in error", "ParserTest");
            for (String model : parserTest.getModelsInError()) {
                Log.debug("  " + model, "ParserTest");
            }
        }
        Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserTest");
        Log.info("Count of correctly parsed models: "
                + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserTest");

        assertTrue("There were models that could not be parsed", parserTest.getModelsInError()
                .isEmpty());
    }

    /**
     * Visits files of the given file ending and checks whether they are parsable.
     *
     * @see Files#walkFileTree(Path, java.nio.file.FileVisitor)
     */
    private static class ParseTest extends SimpleFileVisitor<Path> {

        private String fileEnding;

        private List<String> modelsInError = new ArrayList<>();

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
        public List<String> getModelsInError() {
            return this.modelsInError;
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs)
                throws IOException {
            if (file.toFile().isFile()
                    && (file.toString().toLowerCase().endsWith(fileEnding))) {

                Log.debug("Parsing file " + file.toString(), "ParserTest");
                testCount++;
                Optional<ASTEMACompilationUnit> model = Optional.empty();
                boolean expectingError = ParserTest.expectedParseErrorModels.contains(file.toString());

                EMADLParser parser = new EMADLParser();
                try {
                    if (expectingError) {
                        Log.enableFailQuick(false);
                    }
                    model = parser.parse(file.toString());
                }
                catch (Exception e) {
                    if (!expectingError) {
                        Log.error("Exception during test", e);
                    }
                }
                if (!expectingError && (parser.hasErrors() || !model.isPresent())) {
                    modelsInError.add(file.toString());
                    Log.error("There were unexpected parser errors");
                }
                else {
                    Log.getFindings().clear();
                }
                Log.enableFailQuick(ParserTest.ENABLE_FAIL_QUICK);
            }
            return FileVisitResult.CONTINUE;
        }
    }
}
