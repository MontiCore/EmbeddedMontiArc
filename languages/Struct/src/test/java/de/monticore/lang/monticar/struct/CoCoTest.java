/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct;

import de.monticore.lang.monticar.struct._ast.ASTStructCompilationUnit;
import de.monticore.lang.monticar.struct._cocos.StructCoCoChecker;
import de.monticore.lang.monticar.struct._parser.StructParser;
import de.monticore.lang.monticar.struct._symboltable.StructLanguage;
import de.monticore.lang.monticar.struct.coco.DefaultStructCoCoChecker;
import de.se_rwth.commons.logging.Log;
import org.junit.Assert;
import org.junit.Before;
import org.junit.BeforeClass;
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

import static org.junit.Assert.assertTrue;

public class CoCoTest {

    private StructCoCoChecker checker;

    @BeforeClass
    public static void setupSpec() {
        Log.getFindings().clear();
        Log.enableFailQuick(false);
    }

    @Before
    public void setup() {
        checker = DefaultStructCoCoChecker.create();
    }

    @Test
    public void testValidModels() throws IOException {
        CoCoTester tester = new CoCoTester(
                checker,
                "." + StructLanguage.FILE_ENDING,
                false

        );
        Files.walkFileTree(Paths.get("src/test/resources/test/coco/valid"), tester);
        logEpilogue(tester);
    }

    @Test
    public void testInvalidModels() throws IOException {
        CoCoTester tester = new CoCoTester(
                checker,
                "." + StructLanguage.FILE_ENDING,
                true

        );
        Files.walkFileTree(Paths.get("src/test/resources/test/coco/invalid"), tester);
        logEpilogue(tester);
    }

    private void logEpilogue(CoCoTester tester) {
        if (!tester.getErrors().isEmpty()) {
            Log.debug("Models in error", "CoCoTester");
            for (String model : tester.getErrors()) {
                Log.debug("  " + model, "CoCoTester");
            }
        }
        Log.info(
                "Count of tested models: " + tester.getTestCount(),
                "CoCoTester"
        );
        Log.info(
                "Count of correctly tested models: "
                        + (tester.getTestCount() - tester.getErrors().size()),
                "CoCoTester"
        );
        assertTrue(
                "There were CoCo check failures",
                tester.getErrors().isEmpty()
        );
    }

    private static class CoCoTester extends SimpleFileVisitor<Path> {

        private final StructCoCoChecker checker;
        private final String fileEnding;
        private final boolean isExpectFailure;
        private final List<String> errors = new ArrayList<>();
        private int testCount = 0;

        public CoCoTester(
                StructCoCoChecker checker,
                String fileEnding,
                boolean isExpectFailure
        ) {
            super();
            this.checker = checker;
            this.fileEnding = fileEnding;
            this.isExpectFailure = isExpectFailure;
        }

        public int getTestCount() {
            return this.testCount;
        }

        public List<String> getErrors() {
            return this.errors;
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) throws IOException {
            if (file.toFile().isFile() && (file.toString().toLowerCase().endsWith(fileEnding))) {
                Log.debug("Parsing file " + file.toString(), "CoCoTester");
                testCount++;
                StructParser parser = new StructParser();
                ASTStructCompilationUnit parsedStruct = parser.parse(file.toString()).orElse(null);
                if (parser.hasErrors()) {
                    errors.add(file.toString());
                    Log.error("There were unexpected parser errors");
                }
                Assert.assertNotNull(parsedStruct);
                Log.getFindings().clear();
                checker.checkAll(parsedStruct);
                boolean isSuccess = Log.getFindings().isEmpty();
                boolean isTestFailed = (!isSuccess && !isExpectFailure) || (isSuccess && isExpectFailure);
                if (isTestFailed) {
                    errors.add(file.toString());
                }
            }
            return FileVisitResult.CONTINUE;
        }
    }
}
