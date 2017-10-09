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
package de.monticore.lang.embeddedmontiview.cocos;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTPortCoCo;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTPort;
import de.monticore.lang.monticar.ranges._ast.ASTRange;
import de.monticore.lang.monticar.ranges._ast.ASTRanges;
//import de.monticore.types.types._ast.ASTPrimitiveType;
//import de.monticore.types.types._ast.ASTType;
import de.monticore.lang.monticar.types2._ast.*;
import de.se_rwth.commons.logging.Log;

/**
 * Created by dennisqiao on 2/9/17.
 */
public class PortTypeOnlyBooleanOrSIUnit
    implements EmbeddedMontiViewASTPortCoCo {

  @Override
  public void check(ASTPort node) {
    //might be anonymous:
    if (node.getType().isPresent()) {
      ASTType type = node.getType().get();
      if (!(type instanceof ASTRange) && !(type instanceof ASTRanges) && (type instanceof ASTPrimitiveType && !type.toString().equals("Boolean"))) {
        Log.error(String.format("0xAE753 Port type can only be Boolean or a SI_Unit"), node.get_SourcePositionStart());
      }
    }
  }
}
