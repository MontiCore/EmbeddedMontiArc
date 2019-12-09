/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

public class CircularDependencyCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends LanguageCoCo> getContextCondition() {
        return CircularDependencyCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }
}
