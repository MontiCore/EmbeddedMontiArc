/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import java.util.Arrays;
import java.util.List;

public class CircularDependencyCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends IDECoCo> getContextCondition() {
        return CircularDependencyCoCo.class;
    }

    @Override
    protected List<Integer> getExpectedViolations() {
        return Arrays.asList(1, 3, 1, 1);
    }

    @Override
    protected List<String> getTestCases() {
        return Arrays.asList(
                "CircularDependencyIDE", "CircularDependencyTask",
                "CircularDependencyConfiguration", "CircularDependencyModule"
        );
    }
}
