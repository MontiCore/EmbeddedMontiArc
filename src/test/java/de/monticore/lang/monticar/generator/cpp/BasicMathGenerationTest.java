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
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.ExpandedComponentInstanceSymbol;
import de.monticore.lang.monticar.generator.AbstractSymtabTest;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.List;

import static org.junit.Assert.assertNotNull;

/**
 * @author Sascha Schneiders
 */
public class BasicMathGenerationTest extends AbstractSymtabTest {

    @Test
    public void testRowVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.rowVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }


    @Test
    public void testColumnVectorMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.columnVectorMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testStaticVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.staticMathSectionVariableTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixConstantVariableMathSectionTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixConstantVariableMathSectionTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
    @Test
    public void testMatrixAssignmentTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixAssignmentTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testMatrixArrayPortAccessTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.matrixArrayPortAccessTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }

    @Test
    public void testTypeDimensionsTest() throws IOException {
        TaggingResolver symtab = createSymTabAndTaggingResolver("src/test/resources");

        ExpandedComponentInstanceSymbol componentSymbol = symtab.<ExpandedComponentInstanceSymbol>resolve("test.math.typeDimensionsTest", ExpandedComponentInstanceSymbol.KIND).orElse(null);
        assertNotNull(componentSymbol);
        GeneratorCPP generatorCPP = new GeneratorCPP();
        generatorCPP.useArmadilloBackend();
        generatorCPP.setGenerationTargetPath("./target/generated-sources-cpp/testMath/l0");
        List<File> files = generatorCPP.generateFiles(componentSymbol, symtab);
        String restPath = "testMath/l0/";
        testFilesAreEqual(files, restPath);
    }
}
