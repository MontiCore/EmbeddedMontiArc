/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class NodeModulesImplTests {
    @Mock NotificationService notifications;
    @Mock NPMPackageFactory factory;
    @Mock NPMPackageService service;

    final File nodeModulesFile = Paths.get("src/test/resources/NodeModulesImpl").toFile();
    final File packageFile = new File(nodeModulesFile, "package.json");

    NPMPackage pack;
    NodeModulesImpl nodeModules;

    @BeforeEach
    void before() throws IOException {
        pack = new NPMPackageImpl(packageFile, service);

        when(factory.create(packageFile)).thenReturn(pack);

        nodeModules = new NodeModulesImpl(nodeModulesFile, notifications, factory);
    }

    @Test
    void testGetPath() {
        assertEquals(nodeModulesFile, nodeModules.getPath(), "Paths do not match.");
    }

    @Test
    void testGetPackages() {
        List<NPMPackage> result = nodeModules.getPackages();

        assertEquals(1, result.size(), "There should be exactly one package.");
        assertTrue(result.contains(pack), "Mocked package not included in collection.");
    }
}
