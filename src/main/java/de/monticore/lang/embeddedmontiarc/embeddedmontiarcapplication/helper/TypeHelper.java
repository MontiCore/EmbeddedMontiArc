/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

/**
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
