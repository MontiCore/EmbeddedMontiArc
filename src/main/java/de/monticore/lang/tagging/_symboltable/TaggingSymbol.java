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
package de.monticore.lang.tagging._symboltable;

import java.util.Collection;
import java.util.Map;

import de.monticore.symboltable.CommonScope;
import de.monticore.symboltable.CommonSymbol;
import de.monticore.symboltable.MutableScope;
import de.monticore.symboltable.Symbol;
import de.monticore.symboltable.SymbolKind;
import de.monticore.symboltable.resolving.ResolvingFilter;

/**
 * Created by Michael von Wenckstern on 03.06.2016.
 *
 * @author Michael von Wenckstern
 */
public class TaggingSymbol extends CommonSymbol implements IsTaggable {
  protected MutableScope scope = null;

  /**
   * @see CommonSymbol#CommonSymbol(String, SymbolKind)
   */
  public TaggingSymbol(String name, SymbolKind kind) {
    super(name, kind);
  }

  // do it lazy b/c most symbols will not have tags
  protected MutableScope getScope() {
    if (scope == null) {
      scope = new CommonScope();
      for (ResolvingFilter rf : this.getEnclosingScope().getResolvingFilters()) {
        scope.addResolver(rf);
      }
      ((MutableScope) this.getEnclosingScope()).addSubScope(scope);
    }
    return scope;
  }

  /**
   * if component a/Controller.arc is loaded, then only tags in the
   * files a/*.tag are loaded
   * but this allows you to save different tagging information for the
   * same component in different files, e.g. a/PowerConsumption.tag
   * and a/Time.tag
   *
   * @return all tags of the components
   */
  public Collection<TagSymbol> getTags() {
    return getScope().<TagSymbol>resolveLocally(TagSymbol.KIND);
  }

  /**
   * returns only the tag of a special tag kind (e.g. if you want only
   * to evaluate PowerConsumption of a component, than call
   * getTags(PowerConsumption.TAGKIND)
   * --> methodology is the same as in PN's resolve function
   */
  public <T extends TagSymbol> Collection<T> getTags(final TagKind tagKind) {
    return getScope().<T>resolveLocally(tagKind);
  }

  /**
   * adds a tag to the symbol
   *
   * @param tag the tag symbol which should be added
   */
  public void addTag(final TagSymbol tag) {
    final Map<String, Collection<Symbol>> localSymbols = getScope().getLocalSymbols();
    if ((localSymbols.get(tag.getName()) == null) || !localSymbols.get(tag.getName()).contains(tag)) {
      getScope().add(tag);
    }
  }

  /**
   * adds all tags to the symbol
   *
   * @param tags
   */
  public void addTags(final TagSymbol... tags) {
    for (final TagSymbol tag : tags) {
      addTag(tag);
    }
  }

  /**
   * add all tags to the symbol
   *
   * @param tags
   */
  public void addTags(Iterable<? extends TagSymbol> tags) {
    for (TagSymbol tag : tags) {
      addTag(tag);
    }
  }
}
