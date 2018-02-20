package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.symboltable.CommonSymbol;
import de.monticore.lang.montisim.simlang.util.Time;
import de.monticore.lang.montisim.simlang.util.TimeAlternatives;

public class TimeSymbol extends CommonSymbol{

  public static final TimeSymbolKind KIND = TimeSymbolKind.INSTANCE;
  private TimeAlternatives time;

  public TimeSymbol(String name, TimeAlternatives time) {
    super(name, KIND);
    this.time = time;
  }

  public TimeAlternatives getTime() {
    return time;
  }
}
