package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.simlang.util.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class GravitySymbol extends CommonSymbol{

  public static final GravitySymbolKind KIND = GravitySymbolKind.INSTANCE;
  private AlternativeInput gravity;

  public GravitySymbol(String name, AlternativeInput gravity) {
    super(name, KIND);
    this.gravity = gravity;
  }

  public AlternativeInput getGravity() {
    return gravity;
  }
}
