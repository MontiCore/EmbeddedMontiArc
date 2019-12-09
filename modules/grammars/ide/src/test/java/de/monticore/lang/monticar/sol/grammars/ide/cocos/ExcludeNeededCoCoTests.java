/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import java.util.Collections;
import java.util.List;

public class ExcludeNeededCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends IDECoCo> getContextCondition() {
        return ExcludeNeededCoCo.class;
    }

    @Override
    protected List<Integer> getExpectedViolations() {
        return Collections.singletonList(1);
    }
}
