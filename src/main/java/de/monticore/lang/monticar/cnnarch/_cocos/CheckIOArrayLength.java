/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
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
