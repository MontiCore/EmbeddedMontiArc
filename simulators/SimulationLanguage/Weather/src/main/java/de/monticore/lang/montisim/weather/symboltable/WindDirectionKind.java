/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.weather.symboltable;

import de.monticore.symboltable.SymbolKind;

public class WindDirectionKind implements SymbolKind {
  public static final WindDirectionKind INSTANCE = new WindDirectionKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.weather.symboltable.WindDirectionKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
