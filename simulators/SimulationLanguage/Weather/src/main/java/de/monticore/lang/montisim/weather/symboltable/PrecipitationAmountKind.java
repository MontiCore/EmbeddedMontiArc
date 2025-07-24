/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.SymbolKind;

public class PrecipitationAmountKind implements SymbolKind {
  public static final PrecipitationAmountKind INSTANCE = new PrecipitationAmountKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.weather.symboltable.PrecipitationAmountKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
