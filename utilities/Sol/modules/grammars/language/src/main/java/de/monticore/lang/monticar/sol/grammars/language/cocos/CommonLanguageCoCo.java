/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CommonCoCo;

public abstract class CommonLanguageCoCo extends CommonCoCo<LanguageCoCoChecker> implements LanguageCoCo {
    protected CommonLanguageCoCo(String errorCode, String schema) {
        super(errorCode, schema);
    }
}
