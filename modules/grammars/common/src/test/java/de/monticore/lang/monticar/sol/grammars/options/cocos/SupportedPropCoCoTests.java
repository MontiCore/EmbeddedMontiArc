/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

public class SupportedPropCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends OptionCoCo> getContextCondition() {
        return SupportedPropCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
