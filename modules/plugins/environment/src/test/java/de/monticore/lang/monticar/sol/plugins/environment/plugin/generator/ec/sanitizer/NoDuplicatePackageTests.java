/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.sanitizer;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstall;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.grammars.environment._ast.EnvironmentMill;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
import de.monticore.mcliterals._ast.ASTStringLiteral;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.InjectMocks;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.mockito.junit.jupiter.MockitoSettings;
import org.mockito.quality.Strictness;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class NoDuplicatePackageTests {
    @Mock NotificationService notifications;

    @InjectMocks NoDuplicatePackage sanitizer;

    List<ASTInstruction> instructions;

    @BeforeEach
    void before() {
        ASTStringLiteral cmake = EnvironmentMill.stringLiteralBuilder().setSource("\"cmake\"").build();
        ASTStringLiteral buildEssential = EnvironmentMill.stringLiteralBuilder().setSource("\"build-essential\"").build();

        ASTInstall firstInstall = EnvironmentMill.installBuilder()
                .addPackage(cmake).addPackage(buildEssential).build();

        ASTInstall secondInstall = EnvironmentMill.installBuilder()
                .addPackage(cmake).build();

        instructions = new ArrayList<>();

        instructions.add(firstInstall);
        instructions.add(secondInstall);
    }

    @Test
    void testSanitize() {
        sanitizer.sanitize(instructions);

        assertEquals(1, instructions.size(), "Empty INSTALL should have been pruned.");
    }
}
