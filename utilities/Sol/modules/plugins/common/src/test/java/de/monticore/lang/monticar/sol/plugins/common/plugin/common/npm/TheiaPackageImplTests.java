/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm;

import org.json.JSONArray;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class TheiaPackageImplTests extends NPMPackageImplTests {
    TheiaPackage pack;

    @BeforeEach
    void before() throws IOException {
        super.before();

        pack = new TheiaPackageImpl(super.pack);
    }

    @Test
    void testIsTheiaPackage() {
        assertTrue(getPackage().isTheiaPackage(), "Package should be a Theia package.");
    }

    @Test
    void testIsSolPackage() {
        assertFalse(getPackage().isSolPackage(), "Package should not be a Sol package.");
    }

    @Test
    void testGetAsSolPackage() {
        Optional<SolPackage> solPackage = getPackage().getAsSolPackage();

        assertFalse(solPackage.isPresent(), "Package should not have been returned as Sol package.");
    }

    @Test
    void testGetAsTheiaPackage() {
        Optional<TheiaPackage> theiaPackage = getPackage().getAsTheiaPackage();

        assertTrue(theiaPackage.isPresent(), "Package should have been returned as Theia package.");
    }

    @Test
    void testGetExtensions() {
        JSONArray expected = new JSONArray();
        JSONArray actual = getPackage().getExtensions();

        assertEquals(expected.toString(), actual.toString(), "Extensions do not match.");
    }

    protected TheiaPackage getPackage() {
        return this.pack;
    }

    protected File getPath() {
        return Paths.get("src/test/resources/TheiaPackageImpl/package.json").toFile();
    }
}
