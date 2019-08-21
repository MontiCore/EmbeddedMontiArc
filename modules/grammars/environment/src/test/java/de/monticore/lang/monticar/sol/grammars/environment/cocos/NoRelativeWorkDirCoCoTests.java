/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoRelativeWorkDirCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return NoRelativeWorkDirCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
