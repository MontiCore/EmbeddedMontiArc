/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoAPTGetInstallCoCoTests extends AbstractCoCoTests {
    @Override
    protected EnvironmentCoCo getContextCondition() {
        return new NoAPTGetInstallCoCo();
    }

    @Override
    protected String getExpectedErrorCode() {
        return "ENV0001";
    }

    @Override
    protected String getExpectedErrorMessage() {
        return "ENV0001 Please use 'INSTALL <package> [,<package>]' instead of 'RUN apt-get install'.";
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
