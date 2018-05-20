/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2018, Software Engineering Group at RWTH Aachen,
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
package de.monticore.lang.mathopt._parser;

import de.monticore.lang.math.ParserMathTest;
import de.monticore.lang.mathopt._ast.ASTMathOptCompilationUnit;
import de.se_rwth.commons.logging.Log;

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.Optional;

import static org.junit.Assert.assertTrue;

/**
 * Tests for MontiMathOpt's parser
 * @author Christoph Richter
 */
public class ParserMathOptTest extends ParserMathTest {

    @Override
    /**
     * Test parser for all models in path "src/test/resources/"
     */
    protected void test(String fileEnding) throws IOException {
        ParseMathOptTest parserTest = new ParseMathOptTest("." + fileEnding);
        Files.walkFileTree(Paths.get("src/test/resources/"), parserTest);

        if (!parserTest.getModelsInError().isEmpty()) {
            Log.debug("Models in error", "ParserMathOptTest");
            for (String model : parserTest.getModelsInError()) {
                Log.debug("  " + model, "ParserMathOptTest");
            }
        }
        Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserMathOptTest");
        Log.info("Count of correctly parsed models: "
                + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserMathOptTest");

        assertTrue("There were models that could not be parsed", parserTest.getModelsInError().isEmpty());
    }

    protected class ParseMathOptTest extends ParseTest {

        public ParseMathOptTest(String fileEnding) {
            super(fileEnding);
        }

        @Override
        public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) {
            if (file.toFile().isFile()
                    && (file.toString().toLowerCase().endsWith(fileEnding))) {

                Log.debug("Parsing file " + file.toString(), "ParserStreamTest");
                testCount++;
                Optional<ASTMathOptCompilationUnit> compilationUnit = Optional.empty();
                boolean expectingError = ParserMathOptTest.expectedParseErrorModels.contains(file.toString());

                MathOptParser parser = new MathOptParser();
                try {
                    if (expectingError) {
                        Log.enableFailQuick(false);
                    }
                    compilationUnit = parser.parse(file.toString());
                } catch (Exception e) {
                    if (!expectingError) {
                        Log.error("Exception during test", e);
                    }
                }
                if (!expectingError && (parser.hasErrors() || !compilationUnit.isPresent())) {
                    getModelsInError().add(file.toString());
                    Log.error("There were unexpected parser errors");
                } else {
                    Log.getFindings().clear();
                }
                Log.enableFailQuick(ParserMathTest.ENABLE_FAIL_QUICK);
            }
            return FileVisitResult.CONTINUE;
        }
    }
}
