package de.monticore.lang.montisim.simlang._symboltable;

import java.util.Collection;
import java.util.Optional;

public class WeatherSymbol extends de.monticore.symboltable.CommonSymbol {

  public static final WeatherSymbolKind KIND = WeatherSymbolKind.INSTANCE;

  public WeatherSymbol(String name) {
    super(name, KIND);
  }
}
