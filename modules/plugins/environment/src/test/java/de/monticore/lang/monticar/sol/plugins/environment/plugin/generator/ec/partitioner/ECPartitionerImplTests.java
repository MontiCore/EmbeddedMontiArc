/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator.ec.partitioner;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTArg;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTEnv;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstall;
import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTInstruction;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.notification.NotificationService;
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
import static org.mockito.Mockito.doCallRealMethod;
import static org.mockito.Mockito.mock;

@ExtendWith(MockitoExtension.class)
@MockitoSettings(strictness = Strictness.LENIENT)
public class ECPartitionerImplTests {
    @Mock NotificationService notifications;

    @InjectMocks
    ECPartitionerImpl partitioner;

    List<ASTInstruction> instructions = new ArrayList<>();

    @BeforeEach
    void before() {
        ASTArg arg = mock(ASTArg.class);
        ASTInstall installation = mock(ASTInstall.class);
        ASTEnv env = mock(ASTEnv.class);

        doCallRealMethod().when(arg).getType();
        doCallRealMethod().when(installation).getType();
        doCallRealMethod().when(env).getType();

        instructions.add(arg);
        instructions.add(arg);
        instructions.add(arg);

        instructions.add(installation);
        instructions.add(installation);

        instructions.add(env);
    }

    @Test
    void testPartition() {
        List<List<ASTInstruction>> partitions = partitioner.partition(instructions);

        assertEquals(3, partitions.size(), "There should be exactly 3 partitions.");

        for (ASTInstruction instruction : partitions.get(0)) assertEquals("ARG", instruction.getType());
        for (ASTInstruction instruction : partitions.get(1)) assertEquals("INSTALL", instruction.getType());
        for (ASTInstruction instruction : partitions.get(2)) assertEquals("ENV", instruction.getType());
    }
}
