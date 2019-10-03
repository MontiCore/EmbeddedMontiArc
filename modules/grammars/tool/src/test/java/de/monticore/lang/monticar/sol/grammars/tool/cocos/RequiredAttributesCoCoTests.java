/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool.cocos;

public class RequiredAttributesCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ToolCoCo> getContextCondition() {
        return RequiredAttributesCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
