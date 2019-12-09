/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact.cocos;

import java.util.Arrays;
import java.util.List;

public class RequiredAttributesCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends ArtifactCoCo> getContextCondition() {
        return RequiredAttributesCoCo.class;
    }

    @Override
    protected int getExpectedViolations() {
        return 2;
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList("RequiredAttributesProduct", "RequiredAttributesTool", "RequiredAttributesVirtualTool");
    }
}
