/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import java.util.Arrays;
import java.util.List;

public class DuplicateIdentifierCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends IDECoCo> getContextCondition() {
        return DuplicateIdentifierCoCo.class;
    }

    @Override
    protected List<Integer> getExpectedViolations() {
        return Arrays.asList(1, 1);
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("DuplicateIdentifierConfiguration", "DuplicateIdentifierModule");
    }
}
