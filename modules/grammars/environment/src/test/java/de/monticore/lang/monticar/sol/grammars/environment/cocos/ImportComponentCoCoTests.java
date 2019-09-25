/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

public class ImportComponentCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends EnvironmentCoCo> getContextCondition() {
        return ImportComponentCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
