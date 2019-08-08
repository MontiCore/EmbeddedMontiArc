/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

import de.monticore.lang.monticar.sol.grammars.language._ast.ASTLanguageCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.apache.maven.plugin.MojoExecutionException;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.nio.file.Paths;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class LDValidatorImplTests {
    @Mock NotificationService notifications;
    @Mock LanguageClientConfiguration configuration;
    @Mock LanguageCoCoChecker checker;
    @InjectMocks LDValidatorImpl validator;

    @Test
    void testGetPriority() {
        assertTrue(validator.getPriority() < 500, "Priority should be less than the one of NotificationServiceImpl.");
    }

    @Test
    void testOnPluginConfigure() {
        Mojo plugin = mock(Mojo.class);
        File model = Paths.get("src/test/resources/LDValidatorImpl/EmbeddedMontiArcMath.ld").toFile();

        Log.enableFailQuick(false);
        when(configuration.getModels()).thenReturn(Collections.singletonList(model));
        doNothing().when(notifications).info(anyString());
        doNothing().when(notifications).info(anyString(), any());
        doNothing().when(checker).checkAll(any(ASTLanguageCompilationUnit.class));

        assertDoesNotThrow(() -> validator.onPluginConfigure(plugin), "Validator should not throw exception.");

        Log.warn("Some Warning");
        assertThrows(MojoExecutionException.class, () -> validator.onPluginConfigure(plugin), "Validator should throw exception");
    }
}
