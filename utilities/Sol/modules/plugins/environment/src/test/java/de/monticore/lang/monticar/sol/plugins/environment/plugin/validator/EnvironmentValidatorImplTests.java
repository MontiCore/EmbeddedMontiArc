/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validator;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentModelLoader;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTable;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class EnvironmentValidatorImplTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentSymbolTable symbolTable;

    EnvironmentValidatorImpl validator;

    @BeforeAll
    static void beforeAll() {
        Log.enableFailQuick(false);
    }

    @BeforeEach
    void before() {
        Injector injector = Guice.createInjector(new EnvironmentModule());
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources"));
        EnvironmentModelLoader loader = injector.getInstance(EnvironmentModelLoader.class);
        EnvironmentCoCoChecker checker = injector.getInstance(EnvironmentCoCoChecker.class);
        Collection<ASTEnvironmentCompilationUnit> nodes =
                loader.loadModelsIntoScope("EnvironmentValidatorImpl.Root", modelPath);
        ASTEnvironmentCompilationUnit node = nodes.iterator().next();

        when(symbolTable.getRootNode()).thenReturn(Optional.of(node));

        validator = new EnvironmentValidatorImpl(notifications, symbolTable, checker);
    }

    @Test
    void testGetPriority() {
        assertTrue(70 > validator.getPriority(), "Priority not in correct range.");
    }

    @Test
    void testOnPluginExecute() {
        Mojo plugin = mock(Mojo.class);

        assertThrows(RuntimeException.class, () -> validator.onPluginExecute(plugin), "RuntimeException should have been thrown.");
    }
}
