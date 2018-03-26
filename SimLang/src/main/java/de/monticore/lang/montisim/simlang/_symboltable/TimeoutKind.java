package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class TimeoutKind implements SymbolKind {
  public static final TimeoutKind INSTANCE = new TimeoutKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.TimeoutKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
