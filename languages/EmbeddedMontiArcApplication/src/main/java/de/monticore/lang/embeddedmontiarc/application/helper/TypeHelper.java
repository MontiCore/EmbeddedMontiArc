/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.application.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.monticar.resolution._ast.ASTTypeArgument;
import de.monticore.lang.monticar.types2._ast.ASTSimpleReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberTypeArgument;
import de.monticore.symboltable.types.TypeSymbol;
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
