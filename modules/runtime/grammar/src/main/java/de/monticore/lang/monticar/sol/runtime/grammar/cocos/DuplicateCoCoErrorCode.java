/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.grammar.cocos;

import java.util.List;

public class DuplicateCoCoErrorCode extends Exception {
    public DuplicateCoCoErrorCode(List<String> errorCodes) {
        super(String.format("%s have already been used as error codes.", errorCodes));
    }
}
