/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.SymbolKind;

public class LTLVehicleKind implements SymbolKind {
  public static final LTLVehicleKind INSTANCE = new LTLVehicleKind();
  private static final String NAME =
      "de.monticore.lang.montisim. de.monticore.lang.montisim.simlang._symboltable.DetailVehicleKind";

  @Override
  public String getName() {
    return NAME;
  }

  @Override
  public boolean isKindOf(SymbolKind kind) {
    return NAME.equals(kind.getName()) || SymbolKind.super.isKindOf(kind);
  }
}
