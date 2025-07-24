/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.SymbolKind;

public class PressureKind implements SymbolKind {
  public static final PressureKind INSTANCE = new PressureKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.weather.symboltable.PressureKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
