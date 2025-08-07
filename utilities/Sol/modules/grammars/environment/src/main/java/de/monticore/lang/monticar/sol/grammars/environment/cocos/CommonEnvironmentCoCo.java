/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment.cocos;

import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CommonCoCo;

public abstract class CommonEnvironmentCoCo extends CommonCoCo<EnvironmentCoCoChecker> implements EnvironmentCoCo {
    protected CommonEnvironmentCoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }
}
