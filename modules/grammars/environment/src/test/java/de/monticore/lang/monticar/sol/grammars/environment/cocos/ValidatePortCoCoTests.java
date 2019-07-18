/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class ValidatePortCoCoTests extends AbstractCoCoTests {
    @Override
    protected EnvironmentCoCo getContextCondition() {
        return new ValidatePortCoCo();
    }

    @Override
    protected String getExpectedErrorCode() {
        return "ENV0005";
    }

    @Override
    protected String getExpectedErrorMessage() {
        return "ENV0005 Port '0' should be between 1024 and 65535.";
    }

    @Override
    protected Object[] getMessageParameters() {
        return new Object[] { 0 };
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
