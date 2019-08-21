/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
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
