package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class MaxSectorUsersSymbolKind implements SymbolKind {
  public static final MaxSectorUsersSymbolKind INSTANCE = new MaxSectorUsersSymbolKind();
  private static final String NAME = "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.MaxSectorUsersSymbolKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
