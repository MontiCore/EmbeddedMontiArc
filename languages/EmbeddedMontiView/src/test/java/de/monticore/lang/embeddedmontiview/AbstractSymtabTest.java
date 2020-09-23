/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview;

import de.monticore.ModelingLanguageFamily;
import de.monticore.io.paths.ModelPath;
//import de.monticore.java.lang.JavaDSLLanguage;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.EmbeddedMontiViewLanguage;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewComponentSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewExpandedComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable.ViewSymbol;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.Scope;

import java.nio.file.Paths;
import java.util.Collection;
import java.util.Optional;

/**
 * Common methods for symboltable tests
 *
 */
public class AbstractSymtabTest {
  protected static Scope createSymTab(String viewCmpName, String... modelPath) {
    ModelingLanguageFamily fam = new ModelingLanguageFamily();
    EmbeddedMontiViewLanguage montiViewLanguage = new EmbeddedMontiViewLanguage();

    fam.addModelingLanguage(montiViewLanguage);
    // TODO should we use JavaDSLLanguage or add the resolvers in MALang?
//    fam.addModelingLanguage(new JavaDSLLanguage());

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
