/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.parser;

import de.monticore.lang.monticar.sol.grammars.option._parser.OptionParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.*;

public class OptionParserTests {
    OptionParser parser = new OptionParser();

    @BeforeAll
    static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    void before() {
        Log.getFindings().clear();
    }

    protected String fetchContent(String type, String validity) throws IOException {
        File file = Paths.get(String.format("src/test/resources/parser/%s/%s.option", validity, type)).toFile();

        return FileUtils.readFileToString(file, StandardCharsets.UTF_8);
    }

    @Test
    void testOptionParser() throws IOException {
        String validContent = this.fetchContent("Instance", "valid");
        String invalidContent = this.fetchContent("Instance", "invalid");

        assertTrue(parser.parse_StringOption(validContent).isPresent(), "AST should be present.");
        assertEquals(0, Log.getFindings().size(), "There should be no errors.");

        assertFalse(parser.parse_StringOption(invalidContent).isPresent(), "AST should not be present.");
        assertNotEquals(0, Log.getFindings().size(), "There should be errors.");
    }

    @Test
    void testOptionTypeParser() throws IOException {
        String validContent = this.fetchContent("Type", "valid");
        String invalidContent = this.fetchContent("Type", "invalid");

        assertTrue(parser.parse_String(validContent).isPresent(), "AST should be present.");
        assertEquals(0, Log.getFindings().size(), "There should be no errors.");

        assertFalse(parser.parse_String(invalidContent).isPresent(), "AST should not be present.");
        assertNotEquals(0, Log.getFindings().size(), "There should be errors.");
    }
}
