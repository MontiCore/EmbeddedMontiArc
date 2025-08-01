/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.SymbolKind;

public class WeatherPhenomenaKind implements SymbolKind {
  public static final WeatherPhenomenaKind INSTANCE = new WeatherPhenomenaKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.weather.symboltable.WeatherPhenomenaKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
