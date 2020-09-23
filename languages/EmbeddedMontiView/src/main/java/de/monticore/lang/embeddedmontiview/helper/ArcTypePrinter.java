/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.helper;

import de.monticore.lang.embeddedmontiview.embeddedmontiview.types.TypesPrinter;
//import de.monticore.types.types._ast.ASTReferenceType;
//import de.monticore.types.types._ast.ASTType;
import de.monticore.lang.monticar.types2._ast.ASTReferenceType;
import de.monticore.lang.monticar.types2._ast.ASTType;
//import de.se_rwth.commons.logging.Log;

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

}
