package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class GravitySymbol extends CommonSymbol{

  public static final GravitySymbolKind KIND = GravitySymbolKind.INSTANCE;
  private ValueListRangeLambda gravity;

  public GravitySymbol(String name, ValueListRangeLambda gravity) {
    super(name, KIND);
    this.gravity = gravity;
  }

  public ValueListRangeLambda getGravity() {
    return gravity;
  }
}
