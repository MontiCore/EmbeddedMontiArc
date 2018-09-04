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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class TypeHelper {

    public static boolean isRangeType(TypeSymbol typeSymbol) {
        return typeSymbol.getName().equals("RangeType");
    }


    public static boolean isUnitNumberTypeArgument(TypeSymbol typeSymbol) {
        return typeSymbol.getName().equals("UnitNumberTypeArgument");
    }

    public static int getUnitNumberFromUnitNumberTypeArgument(ASTSubComponent node, int index) {
        int counter = 0;
        //TODO if UnitNumberResolution consumes port name, use this
        if (node.getType() instanceof ASTSimpleReferenceType) {
            if (((ASTSimpleReferenceType) node.getType()).getTypeArgumentsOpt().isPresent()) {
                for (ASTTypeArgument typeArgument : ((ASTSimpleReferenceType) node.getType()).getTypeArguments().getTypeArgumentList()) {
                    if (typeArgument instanceof ASTUnitNumberTypeArgument) {
                        ASTUnitNumberTypeArgument unitNumberTypeArgument = ((ASTUnitNumberTypeArgument) typeArgument);
                        Log.debug("found resolution number " + unitNumberTypeArgument.getNumberWithUnit().getNumber().get().intValue() + "at index " + counter, "information");
                        if (counter == index)
                            return unitNumberTypeArgument.getNumberWithUnit().getNumber().get().intValue();
                        ++counter;
                    }
                }
            }
        }
        return 0;
    }
}
