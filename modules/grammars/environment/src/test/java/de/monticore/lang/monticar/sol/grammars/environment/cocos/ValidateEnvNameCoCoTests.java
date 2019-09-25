/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class ValidateEnvNameCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return ValidateEnvNameCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
