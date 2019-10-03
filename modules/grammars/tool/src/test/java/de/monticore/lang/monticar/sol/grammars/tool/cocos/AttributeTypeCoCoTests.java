/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

public class AttributeTypeCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return AttributeTypeCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
