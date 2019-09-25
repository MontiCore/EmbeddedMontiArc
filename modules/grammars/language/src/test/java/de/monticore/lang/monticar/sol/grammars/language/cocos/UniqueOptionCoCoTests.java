/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

public class UniqueOptionCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends LanguageCoCo> getContextCondition() {
        return UniqueOptionCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
