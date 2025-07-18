/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.parser;

import de.monticore.lang.monticar.sol.grammars.environment._parser.EnvironmentParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.apache.commons.io.filefilter.WildcardFileFilter;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Collection;

import static org.junit.jupiter.api.Assertions.*;

public class EnvironmentParserTests {
    EnvironmentParser parser = new EnvironmentParser();

    @BeforeAll
    static void beforeAll() {
        Log.enableFailQuick(false);
        Log.getFindings().clear();
    }

    @Test
    void testParseString() throws IOException {
        File validFolder = Paths.get("src/test/resources/parser/valid").toFile();
        File invalidFolder = Paths.get("src/test/resources/parser/invalid").toFile();
        WildcardFileFilter filter = new WildcardFileFilter("*.ddf");
        Collection<File> validFiles = FileUtils.listFiles(validFolder, filter, TrueFileFilter.INSTANCE);
        Collection<File> invalidFiles = FileUtils.listFiles(invalidFolder, filter, TrueFileFilter.INSTANCE);

        for (File validFile : validFiles) {
            String validContent = FileUtils.readFileToString(validFile, "UTF-8");

            assertTrue(parser.parse_String(validContent).isPresent(), "AST should be present.");
            assertEquals(0, Log.getFindings().size(), "There should be no errors.");
        }

        for (File invalidFile : invalidFiles) {
            String invalidContent = FileUtils.readFileToString(invalidFile, "UTF-8");

            assertFalse(parser.parse_String(invalidContent).isPresent(), "AST should be present.");
            assertNotEquals(0, Log.getFindings().size(), "There should be no errors.");
        }
    }
}
