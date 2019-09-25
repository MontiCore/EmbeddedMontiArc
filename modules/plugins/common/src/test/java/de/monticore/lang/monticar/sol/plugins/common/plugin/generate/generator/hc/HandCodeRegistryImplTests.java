/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.hc;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.configuration.GeneratePluginConfiguration;
import org.apache.maven.plugin.Mojo;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
public class HandCodeRegistryImplTests {
    Set<Path> handCodes = new HashSet<>();

    @Mock NotificationService notifications;
    @Mock GeneratePluginConfiguration configuration;
    @Mock Mojo plugin;

    @InjectMocks HandCodeRegistryImpl registry;

    @BeforeEach
    void before() {
        Path handCodePath = Paths.get("src/test/resources/HandCodeRegistryImpl");
        List<File> handCodePaths = new ArrayList<>();

        Path fileTypeScript = Paths.get("src/test/resources/HandCodeRegistryImpl/file.ts");
        Path fileJava = Paths.get("src/test/resources/HandCodeRegistryImpl/package/File.java");

        handCodes.add(handCodePath.relativize(fileTypeScript));
        handCodes.add(handCodePath.relativize(fileJava));

        handCodePaths.add(handCodePath.toFile());
        when(configuration.getHandCodedPaths()).thenReturn(handCodePaths);
        registry.onPluginConfigure(plugin);
    }

    @Test
    void testGetHandCodes() {
        assertEquals(2, registry.getHandCodes().size(), "There should be exactly 2 handwritten code files.");
        assertTrue(registry.getHandCodes().containsAll(handCodes), "There are some handwritten code files missing.");
    }

    @Test
    void testGetPriority() {
        assertEquals(100, registry.getPriority(), "Unexpected Priority.");
    }

    @Test
    void testOnPluginConfigure() {
        assertEquals(2, registry.getHandCodes().size(), "There should be exactly 2 handwritten code files.");
        assertTrue(registry.getHandCodes().containsAll(handCodes), "There are some handwritten code files missing.");
    }
}
