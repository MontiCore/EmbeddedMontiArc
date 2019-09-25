/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;
import org.apache.commons.cli.ParseException;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class DefaultOptionsServiceTests {
    @Mock OptionsRegistry registry;

    @InjectMocks DefaultOptionsService service;

    @BeforeEach
    public void before() throws ParseException {
        Options options = new Options();

        options.addOption(new Option("number", true, "A number."));
        options.addOption(new Option("file", true, "A file."));
        options.addOption(new Option("path", true, "A path"));
        options.addOption(new Option("string", true, "A string"));

        when(registry.getOptions()).thenReturn(options);

        service.onConfigure(new String[] {
                "-number", "4",
                "-file", "SomeFile.txt",
                "-path", "some/path",
                "-string", "SomeString"
        });
    }

    @Test
    void testHasOption() {
        assertTrue(service.hasOption("string"), "There should be a string option.");
        assertFalse(service.hasOption("dummy"), "There should not be a dummy option.");
    }

    @Test
    void testGetOptionAsString() {
        assertNotNull(service.getOptionAsString("string"), "Option should be a String.");
        assertEquals("SomeString", service.getOptionAsString("string"), "Values do not match.");
        assertEquals("string", service.getOptionAsString("dummy", "string"), "Option should be 'string'.");
    }

    @Test
    void testGetOptionAsPath() {
        Path path = Paths.get("some/path");
        Path defaultPath = Paths.get("some/other/path");

        assertNotNull(service.getOptionAsPath("path"), "Option should be a Path.");
        assertEquals(path, service.getOptionAsPath("path"), "Values do not match.");
        assertEquals(defaultPath, service.getOptionAsPath("dummy", "some/other/path"), "Option should be 'some/other/path'.");
    }

    @Test
    void testGetOptionAsFile() {
        File file = new File("SomeFile.txt");
        File defaultFile = new File("SomeDefaultFile.txt");

        assertNotNull(service.getOptionAsFile("path"), "Option should be a File.");
        assertEquals(file, service.getOptionAsFile("file"), "Values do not match.");
        assertEquals(defaultFile, service.getOptionAsFile("dummy", "SomeDefaultFile.txt"), "Option should be 'SomeDefaultFile.txt'.");
    }

    @Test
    void testGetOptionAsInteger() {
        assertEquals(4, service.getOptionAsInteger("number"), "Values do not match.");
        assertEquals(5, service.getOptionAsInteger("dummy", "5"), "Option should be '5'.");
    }

    @Test
    void testOnConfigure() {
        assertNotNull(service.commandLine, "CommandLine should be set.");
    }
}
