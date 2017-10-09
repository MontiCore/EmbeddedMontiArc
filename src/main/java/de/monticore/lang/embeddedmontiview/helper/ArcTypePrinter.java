/**
 * ******************************************************************************
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
 * @author Robert Heim
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
