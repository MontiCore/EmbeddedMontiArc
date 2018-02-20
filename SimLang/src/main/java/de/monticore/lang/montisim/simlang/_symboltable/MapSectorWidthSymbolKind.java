package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class MapSectorWidthSymbolKind implements SymbolKind {
  public static final MapSectorWidthSymbolKind INSTANCE = new MapSectorWidthSymbolKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.MapSectorWidthSymbolKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
