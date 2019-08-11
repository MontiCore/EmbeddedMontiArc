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

import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchTypeSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchitectureSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.IODeclarationSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

public class CheckIOType extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchitectureSymbol architecture) {
        for (IODeclarationSymbol ioDeclaration : architecture.getIODeclarations()){
            checkIO(ioDeclaration);
        }
    }

    public void checkIO(IODeclarationSymbol ioDeclaration) {
        ArchTypeSymbol type = ioDeclaration.getType();

        if (type.getDomain().isComplex() || type.getDomain().isBoolean()){
            Log.error("0" + ErrorCodes.INVALID_IO_TYPE + " Invalid IO element type. " +
                    "Type has to be rational or whole number.");
        }

        if (type.getDimensionSymbols().size() > 3){
            Log.error("0" + ErrorCodes.INVALID_IO_TYPE + " Invalid dimension shape. Shape has to be an tuple either of size 1, 2 or 3 (e.g. {number_of_channels, height, width})."
                    , ioDeclaration.getSourcePosition());
        }
        else {
            for (ArchSimpleExpressionSymbol dimension : type.getDimensionSymbols()){
                Optional<Integer> value = dimension.getIntValue();
                if (!value.isPresent() || value.get() <= 0){
                    Log.error("0" + ErrorCodes.INVALID_IO_TYPE + " Invalid shape. " +
                                    "The dimension sizes can only be positive integers."
                            , dimension.getSourcePosition());
                }
            }
        }
    }

}
