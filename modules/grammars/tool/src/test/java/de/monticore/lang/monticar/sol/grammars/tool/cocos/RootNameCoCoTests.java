/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

import java.util.Collections;
import java.util.List;

public class RootNameCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return RootNameCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }

    @Override
    protected List<String> getAdditionalTestCases() {
        return Collections.singletonList("RootNameResources");
    }
}
