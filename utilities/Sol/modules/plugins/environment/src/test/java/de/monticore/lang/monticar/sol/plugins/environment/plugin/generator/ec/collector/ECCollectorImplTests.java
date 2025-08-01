/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.collector;

import com.google.inject.Guice;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.environment.EnvironmentModule;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentModelLoader;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.symboltable.EnvironmentSymbolTable;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.when;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ECCollectorImplTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentSymbolTable symbolTable;

    @InjectMocks
    ECCollectorImpl collector;

    @BeforeEach
    void before() {
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources"));
        EnvironmentModelLoader loader =
                Guice.createInjector(new EnvironmentModule()).getInstance(EnvironmentModelLoader.class);
        Collection<ASTEnvironmentCompilationUnit> nodes =
                loader.loadModelsIntoScope("DDFCollectorImpl.FirstFile", modelPath);
        ASTEnvironmentCompilationUnit node = nodes.iterator().next();

        when(symbolTable.getRootNode()).thenReturn(Optional.of(node));
    }

    @Test
    void testCollect() {
        List<ASTInstruction> instructions = collector.collect();

        assertEquals(11, instructions.size(), "There should be exactly 9 instructions.");
    }
}
