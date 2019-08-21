/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoCommandChainingCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return NoCommandChainingCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
