/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class ValidateEnvNameCoCoTests extends AbstractCoCoTests {
    @Override
    protected EnvironmentCoCo getContextCondition() {
        return new ValidateEnvNameCoCo();
    }

    @Override
    protected String getExpectedErrorCode() {
        return "ENV0004";
    }

    @Override
    protected String getExpectedErrorMessage() {
        return "ENV0004 Environmental Variable 'bad' has not the right name format.";
    }

    @Override
    protected Object[] getMessageParameters() {
        return new Object[] { "bad" };
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
