/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import java.util.Arrays;
import java.util.List;

public class UniqueArtifactCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ArtifactCoCo> getContextCondition() {
        return UniqueArtifactCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 1;
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("UniqueArtifactProduct", "UniqueArtifactTool");
    }
}
