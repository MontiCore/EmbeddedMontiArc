/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import de.monticore.lang.monticar.sol.plugins.common.plugin.common.configuration.PluginConfiguration;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.project.MavenProject;
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
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

import static org.junit.jupiter.api.Assertions.assertIterableEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class NPMPackageServiceImplTests {
    @Mock NotificationService notifications;
    @Mock PluginConfiguration configuration;
    @Mock NodeModulesResolver resolver;
    @Mock NPMPackageFactory factory;

    @InjectMocks NPMPackageServiceImpl service;

    NPMPackage currentPackage;
    NPMPackage somePackage;

    @BeforeEach
    void before() {
        File baseDir = Paths.get("src/test/resources/NPMPackageServiceImpl").toFile();
        MavenProject mavenProject = mock(MavenProject.class);
        Mojo plugin = mock(Mojo.class);
        NodeModules nodeModules = mock(NodeModules.class);

        somePackage = mock(NPMPackage.class);
        currentPackage = mock(NPMPackage.class);

        when(configuration.getMavenProject()).thenReturn(mavenProject);
        when(mavenProject.getBasedir()).thenReturn(baseDir);
        when(resolver.resolve(baseDir)).thenReturn(Collections.singletonList(nodeModules));
        when(factory.create(any(File.class))).thenReturn(currentPackage);
        when(nodeModules.getPackages()).thenReturn(Collections.singletonList(somePackage));

        service.onPluginConfigure(plugin);
    }

    @Test
    void testGetPriority() {
        assertTrue(service.getPriority() < 50000, "Priority is not in the correct range.");
    }

    @Test
    void testResolve() {
        when(somePackage.getName()).thenReturn(Optional.of("SomeName"));

        assertTrue(service.resolve("SomeName").isPresent(), "Service should have found a package.");
    }

    @Test
    void testResolveMany() {
        Predicate<NPMPackage> predicate = pack -> pack.getDependencies().size() > 0;

        when(currentPackage.getDependencies()).thenReturn(Collections.singletonList(somePackage));

        assertTrue(service.resolveMany(predicate).size() > 0, "Service should have found a package.");
    }

    @Test
    void testGetCurrentPackage() {
        SolPackage solPackage = mock(SolPackage.class);

        when(currentPackage.isSolPackage()).thenReturn(true);
        when(currentPackage.getAsSolPackage()).thenReturn(Optional.of(solPackage));

        assertTrue(service.getCurrentPackage().isPresent(), "Current package is not present.");
    }

    @Test
    void testGetAllPackages() {
        List<NPMPackage> packages = Arrays.asList(somePackage, currentPackage);

        assertIterableEquals(packages, service.getAllPackages(), "Found packages do not match.");
    }
}
