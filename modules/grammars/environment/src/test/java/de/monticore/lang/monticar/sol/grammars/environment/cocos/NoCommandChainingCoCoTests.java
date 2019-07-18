/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoCommandChainingCoCoTests extends AbstractCoCoTests {
    @Override
    protected EnvironmentCoCo getContextCondition() {
        return new NoCommandChainingCoCo();
    }

    @Override
    protected String getExpectedErrorCode() {
        return "ENV0003";
    }

    @Override
    protected String getExpectedErrorMessage() {
        return "ENV0003 RUN should have at most one command.";
    }

    @Override
    protected Object[] getMessageParameters() {
        return new Object[0];
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
