/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option.cocos;

import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CommonCoCo;

public abstract class CommonOptionCoCo extends CommonCoCo<OptionCoCoChecker> implements OptionCoCo {
    protected CommonOptionCoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }
}
