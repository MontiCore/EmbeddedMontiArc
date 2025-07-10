/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.common.mp;

import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ModelPathServiceImplTests {
    @Mock SolPackage rootPackage;

    final ModelPathServiceImpl service = new ModelPathServiceImpl();
    final Path somePath = Paths.get("src/test/resources");

    @BeforeEach
    public void before() {
        when(rootPackage.getAllSolDependencies()).thenReturn(new ArrayList<>());
        when(rootPackage.getDirectoryAsPath("models")).thenReturn(Optional.of(somePath));
    }

    @Test
    void testResolve() {
        ModelPath actual = service.resolve(rootPackage);
        ModelPath expected = new ModelPath(somePath);

        assertEquals(expected.toString(), actual.toString(), "Model Paths do not match.");
    }
}
