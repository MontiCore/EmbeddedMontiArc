/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.SymbolKind;

public class PrecipitationTypeKind implements SymbolKind {
  public static final PrecipitationTypeKind INSTANCE = new PrecipitationTypeKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.weather.symboltable.PrecipitationTypeKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
