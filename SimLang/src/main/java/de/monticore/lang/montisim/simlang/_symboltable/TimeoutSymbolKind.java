package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class TimeoutSymbolKind implements SymbolKind {
  public static final TimeoutSymbolKind INSTANCE = new TimeoutSymbolKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.TimeoutSymbolKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
