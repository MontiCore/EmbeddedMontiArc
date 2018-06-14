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
package de.monticore.lang.embeddedmontiarc.application.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.monticar.resolution._ast.ASTTypeArgument;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberTypeArgument;
import de.monticore.symboltable.types.TypeSymbol;
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
        if (node.getType() instanceof ASTSimpleReferenceType) {
            if (((ASTSimpleReferenceType) node.getType()).getTypeArguments().isPresent()) {
                for (ASTTypeArgument typeArgument : ((ASTSimpleReferenceType) node.getType()).getTypeArguments().get().getTypeArguments()) {
                    if (typeArgument instanceof ASTUnitNumberTypeArgument) {
                        ASTUnitNumberTypeArgument unitNumberTypeArgument = ((ASTUnitNumberTypeArgument) typeArgument);
                        Log.debug("found resolution number " + unitNumberTypeArgument.getUnitNumber().getNumber().get().intValue() + "at index " + counter, "information");
                        if (counter == index)
                            return unitNumberTypeArgument.getUnitNumber().getNumber().get().intValue();
                        ++counter;
                    }
                }
            }
        }
        return 0;
    }
}
