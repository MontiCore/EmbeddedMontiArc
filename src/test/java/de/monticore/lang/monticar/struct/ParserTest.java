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
     * @author Robert Heim
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
