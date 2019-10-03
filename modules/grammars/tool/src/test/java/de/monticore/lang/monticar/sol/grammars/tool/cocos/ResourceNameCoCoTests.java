/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

public class ResourceNameCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return ResourceNameCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
