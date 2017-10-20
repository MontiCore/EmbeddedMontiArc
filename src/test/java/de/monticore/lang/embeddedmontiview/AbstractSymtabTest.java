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
package de.monticore.lang.embeddedmontiview;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiViewLanguage;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.monticore.lang.embeddedmontiview.tagging.LatencyTagSchema.LatencyTagSchema;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Optional;

/**
 * Common methods for symboltable tests
 *
 * @author Robert Heim
 */
public class AbstractSymtabTest {
  protected static Scope createSymTab(String viewCmpName, String... modelPath) {
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    EmbeddedMontiViewLanguage montiViewLanguage = new EmbeddedMontiViewLanguage();

    LatencyTagSchema.registerTagTypes(montiViewLanguage);

    fam.addModelingLanguage(montiViewLanguage);
    // TODO should we use JavaDSLLanguage or add the resolvers in MALang?
    fam.addModelingLanguage(new JavaDSLLanguage());

    // TODO how to add java default types?
    final ModelPath mp = new ModelPath(Paths.get("src/main/resources/defaultTypes"));
    for (String m : modelPath) {
      mp.addEntry(Paths.get(m));
    }
    GlobalScope scope = new GlobalScope(mp, fam);

    // resolve all referenced ComponentSymbols
    Optional<ViewSymbol> cmp = scope.resolve(viewCmpName, ViewSymbol.KIND);
    if(cmp.isPresent()) {
      resolveViewInstances(cmp.get().getSpannedScope());
    }

    return scope;
  }

  private static void resolveViewInstances(Scope scope) {
    Collection<ViewComponentSymbol> instances = scope.resolveLocally(ViewComponentSymbol.KIND);

    for(ViewComponentSymbol instance : instances) {
      resolveViewComponentInstances(instance.getSpannedScope());
    }
  }

  private static void resolveViewComponentInstances(Scope scope){
    Collection<ViewExpandedComponentInstanceSymbol> instances = scope.resolveLocally(ViewExpandedComponentInstanceSymbol.KIND);

    for(ViewExpandedComponentInstanceSymbol instance : instances){
      resolveViewComponentInstances(instance.getSpannedScope());
    }
  }
}
