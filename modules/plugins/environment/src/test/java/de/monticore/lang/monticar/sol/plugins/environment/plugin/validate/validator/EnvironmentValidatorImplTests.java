/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.validator;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnvironmentCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.validate.configuration.EnvironmentValidateConfiguration;
import de.se_rwth.commons.logging.Log;
import org.apache.maven.plugin.Mojo;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.io.File;
import java.util.Collections;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class EnvironmentValidatorImplTests {
    @Mock NotificationService notifications;
    @Mock EnvironmentValidateConfiguration configuration;
    @Mock EnvironmentCoCoChecker checker;

    @InjectMocks EnvironmentValidatorImpl validator;

    File rootDirectory = new File("src/test/resources/EnvironmentValidatorImpl");
    File model = new File(rootDirectory, "Invalid.ddf");

    @BeforeEach
    void before() {
        Log.enableFailQuick(false);

        when(configuration.getRootDirectory()).thenReturn(rootDirectory);
    }

    @Test
    void testFail() {
        validator.fail();

        assertTrue(validator.hasFailed, "Validator 'hasFailed' should be true.");
        assertEquals(0, Log.getFindings().size(), "Log should have been cleared");
    }

    @Test
    void testValidate() {
        assertThrows(RuntimeException.class, () -> validator.validate(), "RuntimeException should have been thrown.");
        assertTrue(validator.hasFailed, "Validator 'hasFailed' should be true.");
    }

    @Test
    void testGetPriority() {
        assertEquals(50, validator.getPriority(), "Priorities do not match.");
    }

    @Test
    void testOnPluginExecute() {
        Mojo plugin = mock(Mojo.class);

        assertThrows(RuntimeException.class, () -> validator.onPluginExecute(plugin), "RuntimeException should have been thrown.");
        assertTrue(validator.hasFailed, "Validator 'hasFailed' should be true.");
    }

    @Test
    void testEndVisit() {
        Log.warn("Dummy Warning");

        assertThrows(RuntimeException.class, () -> validator.endVisit(rootDirectory), "RuntimeException should have been thrown.");
        assertTrue(validator.hasFailed, "Validator 'hasFailed' should be true.");

        Log.getFindings().clear();
        validator.hasFailed = false;

        assertDoesNotThrow(() -> validator.endVisit(rootDirectory), "RuntimeException should not have been thrown.");
        assertFalse(validator.hasFailed, "Validator 'hasFailed' should be false.");
    }

    @Test
    void testVisitModel() {
        validator.visitModel(model);

        verify(notifications).info("Checking Model '%s' for Syntax Errors.", model);
    }

    @Test
    void testEndVisitModel() {
        Log.warn("Dummy Warning");

        validator.endVisitModel(model);

        assertTrue(validator.hasFailed, "Validator 'hasFailed' should be true.");
    }

    @Test
    void testVisit() {
        ASTEnvironmentCompilationUnit ast = mock(ASTEnvironmentCompilationUnit.class);
        ASTInstruction instruction = mock(ASTInstruction.class);

        when(ast.getInstructionList()).thenReturn(Collections.singletonList(instruction));

        validator.visit(ast);

        assertTrue(validator.ast.containsInstruction(instruction), "Instruction should have been added to AST.");
    }
}
