/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.parser;

import de.monticore.lang.monticar.sol.grammars.language._parser.LanguageParser;
import de.se_rwth.commons.logging.Log;
import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

public class LanguageParserTests {
    LanguageParser parser = new LanguageParser();

    @BeforeAll
    static void beforeAll() {
        Log.enableFailQuick(false);
        Log.getFindings().clear();
    }

    @Test
    void testParseString() throws IOException {
        File validFile = Paths.get("src/test/resources/parser/Valid.lc").toFile();
        File invalidFile = Paths.get("src/test/resources/parser/Invalid.lc").toFile();
        String validContent = FileUtils.readFileToString(validFile, StandardCharsets.UTF_8);
        String invalidContent = FileUtils.readFileToString(invalidFile, StandardCharsets.UTF_8);

        assertTrue(parser.parse_String(validContent).isPresent(), "AST should be present.");
        assertEquals(0, Log.getFindings().size(), "There should be no errors.");

        assertFalse(parser.parse_String(invalidContent).isPresent(), "AST should not be present.");
        assertNotEquals(0, Log.getFindings().size(), "There should be errors.");
    }
}
