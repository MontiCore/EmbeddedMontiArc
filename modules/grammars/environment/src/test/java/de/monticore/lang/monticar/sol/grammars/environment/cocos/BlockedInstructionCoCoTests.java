/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class BlockedInstructionCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return BlockedInstructionCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
