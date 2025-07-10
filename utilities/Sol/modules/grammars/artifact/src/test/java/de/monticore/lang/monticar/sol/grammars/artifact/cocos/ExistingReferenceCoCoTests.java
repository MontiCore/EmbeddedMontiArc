/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

public class ExistingReferenceCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ArtifactCoCo> getContextCondition() {
        return ExistingReferenceCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }
}
