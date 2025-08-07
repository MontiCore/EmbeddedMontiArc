/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import java.util.Arrays;
import java.util.List;

public class UniqueAttributeCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ArtifactCoCo> getContextCondition() {
        return UniqueAttributeCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("UniqueAttributeProduct", "UniqueAttributeTool");
    }
}
