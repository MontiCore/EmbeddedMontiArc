package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class MapSectorHeightSymbolKind implements SymbolKind {
  public static final MapSectorHeightSymbolKind INSTANCE = new MapSectorHeightSymbolKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.MapSectorHeightSymbolKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
