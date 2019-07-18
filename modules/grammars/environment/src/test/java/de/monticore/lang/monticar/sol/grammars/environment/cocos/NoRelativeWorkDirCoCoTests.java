/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoRelativeWorkDirCoCoTests extends AbstractCoCoTests {
    @Override
    protected EnvironmentCoCo getContextCondition() {
        return new NoRelativeWorkDirCoCo();
    }

    @Override
    protected String getExpectedErrorCode() {
        return "ENV0006";
    }

    @Override
    protected String getExpectedErrorMessage() {
        return "ENV0006 Working Directory 'some/path' should be absolute.";
    }

    @Override
    protected Object[] getMessageParameters() {
        return new Object[] { "some/path" };
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
