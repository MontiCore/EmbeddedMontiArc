package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.ValueListRangeLambda;

public class TimeoutSymbol extends CommonSymbol{

  public static final TimeoutSymbolKind KIND = TimeoutSymbolKind.INSTANCE;
  private ValueListRangeLambda timeout;

  public TimeoutSymbol(String name, ValueListRangeLambda timeout) {
    super(name, KIND);
    this.timeout = timeout;
  }

  public ValueListRangeLambda getTimeout() {
    return timeout;
  }
}
