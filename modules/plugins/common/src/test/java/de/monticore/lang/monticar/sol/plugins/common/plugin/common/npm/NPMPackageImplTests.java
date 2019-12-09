/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

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
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class NPMPackageImplTests {
    @Mock NPMPackageService resolver;
    @Mock NPMPackage dependency;

    NPMPackage pack;

    @BeforeEach
    void before() throws IOException  {
        when(resolver.resolve("dummy2")).thenReturn(Optional.of(dependency));

        pack = new NPMPackageImpl(this.getPath(), resolver);
    }

    @Test
    void testGetPath() {
        assertEquals(this.getPath(), getPackage().getPath(), "Paths do not match.");
    }

    @Test
    void testGetName() {
        Optional<String> name = getPackage().getName();

        assertTrue(name.isPresent() && name.get().equals("dummy"), "Names do not match.");
    }

    @Test
    void testGetDependencies() {
        List<NPMPackage> dependencies = getPackage().getDependencies();

        assertTrue(dependencies.contains(dependency), "Mocked dependency should have been returned.");
    }

    @Test
    void testQuery() {
        assertEquals("dummy", getPackage().query("/name").orElse("unknown"), "Names do not match.");
    }

    @Test
    void testIsTheiaPackage() {
        assertFalse(getPackage().isTheiaPackage(), "Package should not be a Theia package.");
    }

    @Test
    void testIsSolPackage() {
        assertTrue(getPackage().isSolPackage(), "Package should be a Sol package.");
    }

    @Test
    void testGetAsSolPackage() {
        Optional<SolPackage> solPackage = getPackage().getAsSolPackage();

        assertTrue(solPackage.isPresent(), "Package should have been returned as Sol package.");
    }

    @Test
    void testGetAsTheiaPackage() {
        Optional<TheiaPackage> theiaPackage = getPackage().getAsTheiaPackage();

        assertFalse(theiaPackage.isPresent(), "Package should not have been returned as Theia package.");
    }

    protected NPMPackage getPackage() {
        return this.pack;
    }

    protected File getPath() {
        return Paths.get("src/test/resources/NPMPackageImpl/package.json").toFile();
    }
}
