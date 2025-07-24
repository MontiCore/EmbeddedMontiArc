/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

import com.google.inject.Guice;
import com.google.inject.Injector;
import de.monticore.io.paths.ModelPath;
import de.monticore.lang.monticar.sol.grammars.language.LanguageModule;
import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageModelLoader;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.MojoExecutionException;
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

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LDValidatorImplTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock LanguageSymbolTable symbolTable;
    @Mock NPMPackageService packages;

    LDValidatorImpl validator;
    SolPackage solPackage;

    @BeforeEach
    void before() {
        Injector injector = Guice.createInjector(new LanguageModule());
        ModelPath modelPath = new ModelPath(Paths.get("src/test/resources"));
        LanguageModelLoader loader = injector.getInstance(LanguageModelLoader.class);
        LanguageCoCoChecker checker = injector.getInstance(LanguageCoCoChecker.class);
        Collection<ASTLanguageCompilationUnit> nodes =
                loader.loadModelsIntoScope("LDValidatorImpl.Child", modelPath);
        ASTLanguageCompilationUnit node = nodes.iterator().next();

        solPackage = mock(SolPackage.class);

        // Log.initDEBUG();
        Log.enableFailQuick(false);
        Log.getFindings().clear();

        when(symbolTable.getRootNode()).thenReturn(Optional.of(node));
        when(packages.getCurrentPackage()).thenReturn(Optional.of(solPackage));

        validator = new LDValidatorImpl(notifications, symbolTable, checker, packages);
    }

    @Test
    void testGetPriority() {
        assertTrue(validator.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }

    @Test
    void testOnPluginConfigure() throws Exception {
        Mojo plugin = mock(Mojo.class);

        when(solPackage.isTheiaPackage()).thenReturn(true);
        validator.onPluginConfigure(plugin);

        when(solPackage.isTheiaPackage()).thenReturn(false);
        assertThrows(MojoExecutionException.class, () ->  validator.onPluginConfigure(plugin), "Validator should throw exception");
    }
}
