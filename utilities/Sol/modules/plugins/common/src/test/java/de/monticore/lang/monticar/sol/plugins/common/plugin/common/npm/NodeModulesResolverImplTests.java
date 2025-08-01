/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.nio.file.Paths;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class NodeModulesResolverImplTests {
    @Mock NotificationService notifications;
    @Mock NodeModulesFactory factory;
    @Mock NodeModules nodeModules;

    @InjectMocks NodeModulesResolverImpl resolver;

    final File nodeModulesFile = Paths.get("src/test/resources/NodeModulesResolverImpl/node_modules").toFile();
    final File start = new File(nodeModulesFile, "package");

    @BeforeEach
    void before() {
        when(factory.create(nodeModulesFile)).thenReturn(nodeModules);
    }

    @Test
    void testResolve() {
        List<NodeModules> result = resolver.resolve(start);

        assertTrue(result.contains(nodeModules), "Resolver should have found mocked NodeModules.");
    }

    @Test
    void testResolveFirst() {
        Optional<NodeModules> result = resolver.resolveFirst(start);

        assertTrue(result.isPresent(), "Resolver should have found a NodeModules.");
        assertEquals(result.get(), nodeModules, "Resolver should have found mocked NodeModules.");
    }
}
