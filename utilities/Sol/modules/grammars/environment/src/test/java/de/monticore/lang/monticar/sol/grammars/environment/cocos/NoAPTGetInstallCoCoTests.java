/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class NoAPTGetInstallCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return NoAPTGetInstallCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
