/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.parser;

import de.monticore.lang.monticar.sol.grammars.ide._parser.IDEParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.apache.commons.io.filefilter.TrueFileFilter;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;
import java.util.Collection;

import static org.junit.jupiter.api.Assertions.*;

public class IDEParserTests {
    IDEParser parser = new IDEParser();

    @BeforeAll
    static void beforeAll() {
        Log.enableFailQuick(false);
        Log.getFindings().clear();
    }

    @Test
    void testParseString() throws IOException {
        Collection<File> validFiles = FileUtils.listFiles(Paths.get("src/test/resources/parser/valid").toFile(), TrueFileFilter.INSTANCE, TrueFileFilter.INSTANCE);

        for (File validFile : validFiles) {
            String validContent = FileUtils.readFileToString(validFile, StandardCharsets.UTF_8);

            assertTrue(parser.parse_String(validContent).isPresent(), String.format("AST should be present for '%s'.", validFile));
            assertEquals(0, Log.getFindings().size(), "There should be no errors.");
        }

        Collection<File> invalidFiles = FileUtils.listFiles(Paths.get("src/test/resources/parser/invalid").toFile(), TrueFileFilter.INSTANCE, TrueFileFilter.INSTANCE);

        for (File invalidFile : invalidFiles) {
            String invalidContent = FileUtils.readFileToString(invalidFile, StandardCharsets.UTF_8);

            assertFalse(parser.parse_String(invalidContent).isPresent(), String.format("AST should not be present for '%s'.", invalidFile));
            assertNotEquals(0, Log.getFindings().size(), "There should be errors.");
        }
    }
}
