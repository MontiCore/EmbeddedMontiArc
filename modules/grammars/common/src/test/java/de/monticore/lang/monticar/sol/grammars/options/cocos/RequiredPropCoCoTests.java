/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

public class RequiredPropCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends OptionCoCo> getContextCondition() {
        return RequiredPropCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
