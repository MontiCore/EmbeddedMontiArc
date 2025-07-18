/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.monticar.cnnarch._symboltable.*;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckIOArrayLength extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (IODeclarationSymbol ioDeclaration : architecture.getIODeclarations()){
            checkIO(ioDeclaration);
        }
    }

    public void checkIO(IODeclarationSymbol ioDeclaration) {
        if (ioDeclaration.getArrayLength() > IODeclarationSymbol.MAX_ARRAY_LENGTH) {
            Log.error("0" + ErrorCodes.INVALID_IO_ARRAY_LENGTH + " Invalid IO array length. Length can not be bigger than " + IODeclarationSymbol.MAX_ARRAY_LENGTH
                      , ioDeclaration.getSourcePosition());
        }
    }

}
