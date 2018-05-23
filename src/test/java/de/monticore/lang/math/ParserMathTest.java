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
package de.monticore.lang.math;

import de.monticore.antlr4.MCConcreteParser;
import de.monticore.ast.ASTNode;
import de.monticore.lang.math._parser.MathParser;
import de.se_rwth.commons.logging.Log;
import org.antlr.v4.runtime.RecognitionException;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.IOException;
import java.nio.file.*;
import java.nio.file.attribute.BasicFileAttributes;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import static org.junit.Assert.assertTrue;

/**
 * @author Robert Heim / Michael von Wenckstern
 * is copied from MontiArc4/ParserMathTest.java
 */
public class ParserMathTest {

    public static final boolean ENABLE_FAIL_QUICK = false; // otherwise JUnit test will not fail
    private static List<String> expectedParseErrorModels = Arrays.asList(
            "src/test/resources/LayoutError.tag")
            .stream().map(s -> Paths.get(s).toString())
            .collect(Collectors.toList());

    protected MCConcreteParser getParser() {
        return new MathParser();
    }

    protected String getModelsBaseDir() {
        return "src/test/resources/";
    }

    @BeforeClass
    public static void setUp() {
        // ensure an empty log
        Log.getFindings().clear();
        Log.enableFailQuick(ENABLE_FAIL_QUICK);
    }

    @Test
    public void testMath() throws RecognitionException, IOException {
        test("m");
    }

    protected void test(String fileEnding) throws IOException {
        ParseTest parserTest = new ParseTest("." + fileEnding, getParser());
        Files.walkFileTree(Paths.get(getModelsBaseDir()), parserTest);

        if (!parserTest.getModelsInError().isEmpty()) {
            Log.debug("Models in error", "ParserMathTest");
            for (String model : parserTest.getModelsInError()) {
                Log.debug("  " + model, "ParserMathTest");
            }
        }
        Log.info("Count of tested models: " + parserTest.getTestCount(), "ParserMathTest");
        Log.info("Count of correctly parsed models: "
                + (parserTest.getTestCount() - parserTest.getModelsInError().size()), "ParserMathTest");

        assertTrue("There were models that could not be parsed", parserTest.getModelsInError()
                .isEmpty());
    }

    /**
     * Visits files of the given file ending and checks whether they are parsable.
     *
     * @author Robert Heim
     * // PrimaryUnitExpression = PrimaryExpression2 (unit:EMAUnit)?;
     * @see Files#walkFileTree(Path, java.nio.file.FileVisitor)
     */
    private static class ParseTest extends SimpleFileVisitor<Path> {

        private MCConcreteParser parser;

        private String fileEnding;

        private List<String> modelsInError = new ArrayList<>();

        private int testCount = 0;

        public ParseTest(String fileEnding, MCConcreteParser parser) {
            super();
            this.fileEnding = fileEnding;
            this.parser = parser;
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
                Optional<? extends ASTNode> compilationUnit = Optional.empty();
                boolean expectingError = ParserMathTest.expectedParseErrorModels.contains(file.toString());

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
                    modelsInError.add(file.toString());
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
