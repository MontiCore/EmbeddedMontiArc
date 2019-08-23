/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

public class UniqueOptionCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends OptionCoCo> getContextCondition() {
        return UniqueOptionCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
