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

import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.monticore.lang.embeddedmontiview.helper.ArcTypePrinter;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._cocos.EmbeddedMontiViewASTSubComponentCoCo;
import de.monticore.symboltable.Scope;
import de.se_rwth.commons.logging.Log;

import java.util.Optional;

/**
 * Implements R3 and R4 from AHs PhD thesis
 *
 * @author Crispin Kirchner
 */
public class ReferencedSubComponentExists
    implements EmbeddedMontiViewASTSubComponentCoCo {

  /**
   * @see de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentBodyCoCo#check(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponentBody)
   */
  @Override
  public void check(ASTSubComponent node) {
    String referenceName = ArcTypePrinter.printTypeWithoutTypeArgumentsAndDimension(node.getType());

    Scope scope = node.getEnclosingScope().get();

    Optional<ViewComponentSymbol> componentSymbol = scope.<ViewComponentSymbol>resolve(referenceName, ViewComponentSymbol.KIND);

    if (!componentSymbol.isPresent()) {
      Log.error(String.format("0x069B7 Type \"%s\" could not be resolved", referenceName), node.get_SourcePositionStart());
    }
  }

}
