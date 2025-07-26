/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import java.util.Arrays;
import java.util.List;

public class ArtifactNameCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ArtifactCoCo> getContextCondition() {
        return ArtifactNameCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("ArtifactName", "ArtifactNameProduct", "ArtifactNameTool");
    }
}
