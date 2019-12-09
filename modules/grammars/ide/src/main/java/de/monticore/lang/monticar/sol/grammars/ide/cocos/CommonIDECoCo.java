/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CommonCoCo;

public abstract class CommonIDECoCo extends CommonCoCo<IDECoCoChecker> implements IDECoCo {
    protected CommonIDECoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }
}
