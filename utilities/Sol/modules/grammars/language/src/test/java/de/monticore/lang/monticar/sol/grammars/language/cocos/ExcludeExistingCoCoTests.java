/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

public class ExcludeExistingCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends LanguageCoCo> getContextCondition() {
        return ExcludeExistingCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
