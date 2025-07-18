/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class SolPackageImplTests extends NPMPackageImplTests {
    SolPackage pack;
    SolPackage solDependency;

    @BeforeEach
    void before() throws IOException {
        super.before();

        solDependency = mock(SolPackage.class);

        when(dependency.isSolPackage()).thenReturn(true);
        when(dependency.getAsSolPackage()).thenReturn(Optional.of(solDependency));

        pack = new SolPackageImpl(super.pack);
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

    @Test
    void testGetDirectory() {
        assertTrue(getPackage().getDirectory("models").isPresent(), "Directory does not match.");
    }

    @Test
    void testGetDirectoryAsPath() {
        Path expected = Paths.get("src/test/resources/SolPackageImpl/models");
        Path actual = getPackage().getDirectoryAsPath("models").orElse(getPath().toPath());

        assertEquals(expected, actual, "Paths do not match.");
    }

    @Test
    void testGetSolDependencies() {
        assertEquals(1, pack.getSolDependencies().size(), "There should be exactly one Sol dependency.");
    }

    @Test
    void testGetAllSolDependencies() {
        when(solDependency.getAllSolDependencies()).thenReturn(new ArrayList<>());

        assertEquals(1, pack.getAllSolDependencies().size(), "There should be exactly one Sol dependency.");
    }

    protected SolPackage getPackage() {
        return this.pack;
    }

    protected File getPath() {
        return Paths.get("src/test/resources/SolPackageImpl/package.json").toFile();
    }
}
