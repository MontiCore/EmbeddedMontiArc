/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types.TypesPrinter;
import de.monticore.lang.monticar.resolution._ast.ASTUnitNumberTypeArgument;
import de.monticore.types.types._ast.ASTReferenceType;
import de.monticore.types.types._ast.ASTSimpleReferenceType;
import de.monticore.types.types._ast.ASTType;
import de.monticore.types.types._ast.ASTTypeArgument;
import de.se_rwth.commons.logging.Log;

/**
 * TODO: Implement
 *
 */
public class ArcTypePrinter {

    /**
     * Converts an ASTType to a String
     *
     * @param type ASTType to be converted
     * @return String representation of "type"
     */
    public static String printType(ASTType type) {
        // TODO ArcTypes?!
        return TypesPrinter.printType(type);
    }

    /**
     * Converts an ASTReferenceType to a String
     *
     * @param astReferenceType to be converted
     * @return String representation of "type"
     */
    public static String printReferenceType(ASTReferenceType astReferenceType) {
        // TODO ArcTypes?!
        return TypesPrinter.printReferenceType(astReferenceType);
    }


    /**
     * Converts an ASTType to a String, but omits type arguments
     *
     * @param typeBound to be converted
     * @return String representation of "type" without type arguments
     */
    public static String printTypeWithoutTypeArgumentsAndDimension(ASTType typeBound) {
        // TODO ArcTypes?!
        return TypesPrinter.printTypeWithoutTypeArgumentsAndDimension(typeBound);
    }

    public static String printSubComponentName(ASTSubComponent subComponent) {
        String result = "";
        if (subComponent.getType() instanceof ASTSimpleReferenceType) {
            ASTSimpleReferenceType referenceType = (ASTSimpleReferenceType) subComponent.getType();
            String compNameWithoutPackage = referenceType.getNameList().get(referenceType.getNameList().size() - 1);
            if (referenceType.getTypeArgumentsOpt().isPresent()) {
                for (ASTTypeArgument typeArguments : referenceType.getTypeArgumentsOpt().get().getTypeArgumentList()) {
                    Log.debug(typeArguments.toString(), "typeArgs");
                    if (typeArguments instanceof ASTUnitNumberTypeArgument) {
                        ASTUnitNumberTypeArgument unitNumberTypeArgument = (ASTUnitNumberTypeArgument) typeArguments;
                        if (unitNumberTypeArgument.getNumberWithUnit().getNumber().isPresent()) {
                            compNameWithoutPackage += "_" + unitNumberTypeArgument.getNumberWithUnit().getNumber().get().intValue() + "_";
                            return compNameWithoutPackage;
                        } else
                            Log.debug("0xPRSUCONA1", "No Number present!");
                    } else {

                        Log.debug("No further information", "0xPRSUCONA Case not handled!");

                    }
                }

            }
        }
        return printTypeWithoutTypeArgumentsAndDimension(subComponent.getType());
    }
}
