/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import java.util.Arrays;
import java.util.List;

public class RequiredAttributesCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends IDECoCo> getContextCondition() {
        return RequiredAttributesCoCo.class;
    }

    @Override
    protected List<Integer> getExpectedViolations() {
        return Arrays.asList(1, 2, 1);
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("RequiredAttributesConfiguration", "RequiredAttributesIDE", "RequiredAttributesModule");
    }
}
