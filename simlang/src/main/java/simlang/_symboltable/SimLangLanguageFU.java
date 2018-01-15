/*
 * Copyright (c) 2014 RWTH Aachen. All rights reserved.
 *
 * http://www.se-rwth.de/
 */
package simlang._symboltable;
import de.monticore.symboltable.GlobalScope;
import de.monticore.symboltable.ResolvingConfiguration;
//import de.monticore.symboltable.ArtifactScope;
import de.monticore.symboltable.MutableScope;

import java.util.Optional;

public class SimLangLanguage extends SimLangLanguageTOP {
  public static final String FILE_ENDING = "sim";
  
  public SimLangLanguage() {
    super("SimLang Language", FILE_ENDING);

    setModelNameCalculator(new SimLangModelNameCalculator());
  }
  
  @Override
  public Optional<SimLangSymbolTableCreator> getSymbolTableCreator(
      ResolvingConfiguration resolvingConfiguration, MutableScope enclosingScope) {
    return Optional.of(new SimLangSymbolTableCreator(resolvingConfiguration, enclosingScope));
  }

  @Override
  protected SimLangModelLoader provideModelLoader() {
    return new SimLangModelLoader(this);
  }
}
