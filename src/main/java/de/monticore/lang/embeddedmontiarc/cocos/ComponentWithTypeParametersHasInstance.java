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
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * @author (last commit) Crispin Kirchner
 */
public class ComponentWithTypeParametersHasInstance
    implements EmbeddedMontiArcASTComponentCoCo {

  /**
   * @see EmbeddedMontiArcASTComponentCoCo#check(ASTComponent)
   */
  @Override
  public void check(ASTComponent node) {
    EMAComponentSymbol emaComponentSymbol = (EMAComponentSymbol) node.getSymbolOpt().get();

    Collection<EMAComponentInstantiationSymbol> subComponents = emaComponentSymbol.getSubComponents();

    Set<EMAComponentSymbol> instantiatedInnerComponents = subComponents
        .stream()
        .map(instanceSymbol -> instanceSymbol.getComponentType().getReferencedSymbol())
        .filter(symbol -> symbol.hasFormalTypeParameters())
        .collect(Collectors.toSet());

    List<EMAComponentSymbol> notInstantiatedInnerComponents = emaComponentSymbol
        .getInnerComponents()
        .stream()
        .filter(symbol -> symbol.hasFormalTypeParameters())
        .filter(innerComponent -> !instantiatedInnerComponents.contains(innerComponent))
        .collect(Collectors.toList());

    for (EMAComponentSymbol notInstantiatedInnerComponent : notInstantiatedInnerComponents) {
      Log.error(
          String.format(
              "0x79C00 Inner component \"%s\" must have an instance defining its formal type parameters.",
              notInstantiatedInnerComponent.getName()),
          notInstantiatedInnerComponent.getSourcePosition());
    }
  }
}
