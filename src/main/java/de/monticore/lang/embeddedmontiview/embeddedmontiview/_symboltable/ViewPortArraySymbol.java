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
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import de.monticore.lang.embeddedmontiview.embeddedmontiview._ast.ASTPort;
import de.monticore.lang.monticar.si._symboltable.ResolutionDeclarationSymbol;
import de.monticore.lang.monticar.types2._ast.ASTUnitNumberResolution;
import de.monticore.symboltable.*;
import de.monticore.symboltable.types.JTypeSymbol;
import de.monticore.symboltable.types.references.JTypeReference;
import de.se_rwth.commons.logging.Log;
import org.jscience.mathematics.number.Rational;

import java.util.*;
import java.util.stream.Collectors;

/**
 * Symboltable entry for port arrays
 */
public class ViewPortArraySymbol extends ViewPortSymbol {
  public static final PortArraySymbolKind KIND = PortArraySymbolKind.INSTANCE;

  protected Optional<String> nameSizeDependsOn;

  public ViewPortArraySymbol(String name, String nameSizeDependsOn) {
    super(name, KIND);
    this.nameSizeDependsOn = Optional.ofNullable(nameSizeDependsOn);
    Log.debug(getFullName(), "ViewPortArraySymbol ");
    Log.debug(this.nameSizeDependsOn.orElse(null), "set NameSizeDependsOn to:");
  }

  private int dimension = 1;

  public Optional<String> getNameSizeDependsOn() {
    return nameSizeDependsOn;
  }

  public int getDimension() {
    return dimension;
  }

  public void setDimension(int dimension) {
    this.dimension = dimension;
  }

  public List<? extends ViewPortSymbol> getConcretePortSymbols() {
    return getEnclosingScope().<ViewPortSymbol>resolveLocally(ViewPortSymbol.KIND).stream().filter(s -> s.getName().startsWith(this.getName())).collect(Collectors.toList());
  }

  /**
   * starts with 1
   *
   * @param index
   * @return
   */
  public Optional<ViewPortSymbol> getPortSymbolWithIndex(int index) {
    for (ViewPortSymbol viewPortSymbol : getConcretePortSymbols()) {
      if (viewPortSymbol.getName().contains("[" + index + "]")) {
        return Optional.of(viewPortSymbol);
      }
    }
    return Optional.ofNullable(null);
  }

  public void recreatePortArray(ResolutionDeclarationSymbol resDeclSym, EmbeddedMontiViewSymbolTableCreator emastc, ViewComponentSymbol viewComponentSymbol) {
    Log.debug(getName(), "recreate");

    if (getNameSizeDependsOn().isPresent() && getNameSizeDependsOn().get().equals(resDeclSym.getNameToResolve())) {
      int size = -1;
      if (resDeclSym.getASTResolution() instanceof ASTUnitNumberResolution) {
        size = ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).getNumber().get().intValue();
      }
      List<? extends ViewPortSymbol> portSymbols = getConcretePortSymbols();

      ViewPortSymbol firstPort = getPortSymbolWithIndex(1).get();

      int oldSize = portSymbols.size();
      if (size == 0) {
        size = oldSize;
        ((ASTUnitNumberResolution) resDeclSym.getASTResolution()).setNumber(Rational.valueOf("" + oldSize));
      }
      Log.debug(oldSize + "", "old Port Size:");
      Log.debug(size + "", "new Port Size:");

      for (int i = 0; i <= size; ++i) {
        if (oldSize < i) {
          //Log.debug();
          createPortSymbolForArrayIndex(viewComponentSymbol, (ASTPort) firstPort.getAstNode().get(), this.getName() + "[" + i + "]", firstPort.getStereotype(), firstPort.getTypeReference(), emastc);
        }
      }
      for (int i = size + 1; i <= oldSize; ++i) {
        if (getPortSymbolWithIndex(i).isPresent())
          getEnclosingScope().getAsMutableScope().remove(getPortSymbolWithIndex(i).get());
      }
    }
    else {
      Log.debug("Is not Present", "NameSizeDependsOn:");
    }
  }

  private void createPortSymbolForArrayIndex(ViewComponentSymbol viewComponentSymbol, ASTPort node, String name, Map<String, Optional<String>> stereoType, Optional<JTypeReference<? extends JTypeSymbol>> typeRef, EmbeddedMontiViewSymbolTableCreator emastc) {
    ViewPortSymbol ps = new ViewPortSymbol(name);

    ps.setTypeReference(typeRef);
    ps.setDirection(node.isIncoming());

    stereoType.forEach(ps::addStereotype);

    getEnclosingScope().getAsMutableScope().add(ps);

    emastc.addToScopeAndLinkWithNode(ps, node);

    Log.debug(name + " " + viewComponentSymbol.getAllIncomingPorts().size(), "Added ViewPortSymbol From PortArray:");
  }

  public static class PortArraySymbolKind implements SymbolKind {

    public static final PortArraySymbolKind INSTANCE = new PortArraySymbolKind();

    protected PortArraySymbolKind() {

    }
  }
}
