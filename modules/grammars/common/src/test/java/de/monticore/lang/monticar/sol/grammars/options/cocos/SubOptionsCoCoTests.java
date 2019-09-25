/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

public class SubOptionsCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends OptionCoCo> getContextCondition() {
        return SubOptionsCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
