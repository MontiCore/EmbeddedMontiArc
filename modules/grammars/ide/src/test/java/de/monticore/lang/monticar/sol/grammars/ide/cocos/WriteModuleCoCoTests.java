/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import java.util.Collections;
import java.util.List;

public class WriteModuleCoCoTests extends AbstractCoCoTests {
    @Override
    protected Class<? extends IDECoCo> getContextCondition() {
        return WriteModuleCoCo.class;
    }

    @Override
    protected List<Integer> getExpectedViolations() {
        return Collections.singletonList(1);
    }
}
