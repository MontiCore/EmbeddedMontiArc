/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

public class UndeclareExistingCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends LanguageCoCo> getContextCondition() {
        return UndeclareExistingCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
