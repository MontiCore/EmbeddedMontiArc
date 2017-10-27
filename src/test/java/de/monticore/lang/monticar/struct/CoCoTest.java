/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
