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
/* generated by template templates.de.monticore.lang.tagschema.SimpleTagTypeCreator*/


package de.monticore.lang.embeddedmontiarc.tagging.adaptable;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.PortSymbol;
import de.monticore.lang.tagging._ast.ASTNameScope;
import de.monticore.lang.tagging._ast.ASTScope;
import de.monticore.lang.tagging._ast.ASTTag;
import de.monticore.lang.tagging._ast.ASTTaggingUnit;
import de.monticore.lang.tagging._symboltable.TagSymbolCreator;
import de.monticore.lang.tagging._symboltable.TaggingResolver;
import de.monticore.symboltable.Scope;
import de.monticore.symboltable.Symbol;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * created by SimpleTagTypeCreator.ftl
 */
public class AdaptableSymbolCreator implements TagSymbolCreator {

  public void create(ASTTaggingUnit unit, TaggingResolver tagging) {
    if (unit.getQualifiedNames().stream()
            .map(q -> q.toString())
            .filter(n -> n.endsWith("AdaptableTagSchema"))
            .count() == 0) {
      return; // the tagging model is not conform to the traceability tagging schema
    }
    final String packageName = Joiners.DOT.join(unit.getPackage());
    final String rootCmp = // if-else does not work b/c of final (required by streams)
            (unit.getTagBody().getTargetModel().isPresent()) ?
                    Joiners.DOT.join(packageName, ((ASTNameScope) unit.getTagBody().getTargetModel().get())
                            .getQualifiedNameString().toString()) :
                    packageName;

    for (ASTTag element : unit.getTagBody().getTags()) {
      element.getTagElements().stream()
              .filter(t -> t.getName().equals("Adaptable")) // after that point we can throw error messages
              .forEachOrdered(b ->
                      element.getScopes().stream()
                              .filter(this::checkScope)
                              .map(s -> (ASTNameScope) s)
                              .map(s -> tagging.resolve(Joiners.DOT.join(rootCmp, // resolve down does not try to reload symbol
                                      s.getQualifiedNameString()), PortSymbol.KIND))
                              .filter(Optional::isPresent) // if the symbol is not present, does not mean that the symbol
                              .map(Optional::get)          // is not available at all, maybe it will be loaded later
                              .forEachOrdered(s -> {
                                tagging.addTag(s, new AdaptableSymbol());
                                if(s.isKindOf(PortSymbol.KIND)){
                                  ((PortSymbol)s).setConfig(true);
                                }
                              }));
    }
  }

  public static Scope getGlobalScope(final Scope scope) {
    Scope s = scope;
    while (s.getEnclosingScope().isPresent()) {
      s = s.getEnclosingScope().get();
    }
    return s;
  }

  public void create(ASTTaggingUnit unit, Scope gs) {

    }

  protected PortSymbol checkKind(Collection<Symbol> symbols) {
    PortSymbol ret = null;
    for (Symbol symbol : symbols) {
      if (symbol.getKind().isSame(PortSymbol.KIND)) {
        if (ret != null) {
          Log.error(String.format("0xA4095 Found more than one symbol: '%s' and '%s'",
              ret, symbol));
          return null;
        }
        ret = (PortSymbol)symbol;
      }
    }
    if (ret == null) {
      Log.error(String.format("0xT0001 Invalid symbol kinds: %s. tagTypeName expects as symbol kind 'PortSymbol.KIND'.",
          symbols.stream().map(s -> "'" + s.getKind().toString() + "'").collect(Collectors.joining(", "))));
      return null;
    }
    return ret;
  }

  protected boolean checkScope(ASTScope scope) {
    if (scope.getScopeKind().equals("NameScope")) {
      return true;
    }
    Log.error(String.format("0xT0005 Invalid scope kind: '%s'. Adaptable expects as scope kind 'NameScope'.",
        scope.getScopeKind()), scope.get_SourcePositionStart());
    return false;
  }
}