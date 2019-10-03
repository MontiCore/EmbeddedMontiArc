/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import de.monticore.symboltable.resolving.ResolvedSeveralEntriesException;
import org.junit.jupiter.api.Test;

import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertThrows;

public class UniqueAttributeCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return UniqueAttributeCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }

    @Override @Test
    protected void testCheck() {
        assertThrows(ResolvedSeveralEntriesException.class, super::testCheck);
    }

    @Override
    protected List<String> getAdditionalTestCases() {
        return Collections.singletonList("UniqueAttributeResources");
    }
}
