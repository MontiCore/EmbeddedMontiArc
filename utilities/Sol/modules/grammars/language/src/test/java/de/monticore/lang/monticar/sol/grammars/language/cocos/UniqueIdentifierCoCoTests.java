/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

public class UniqueIdentifierCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends LanguageCoCo> getContextCondition() {
        return UniqueIdentifierCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
