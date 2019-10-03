/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import java.util.Collections;
import java.util.List;

public class ValidAttributeCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return ValidAttributeCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }

    @Override
    protected List<String> getAdditionalTestCases() {
        return Collections.singletonList("ValidAttributeResources");
    }
}
