/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class TimeoutSymbol extends CommonSymbol{

  public static final TimeoutKind KIND = TimeoutKind.INSTANCE;
  private AlternativeInput timeout;

  public TimeoutSymbol(String name, AlternativeInput timeout) {
    super(name, KIND);
    this.timeout = timeout;
  }

  public AlternativeInput getTimeout() {
    return timeout;
  }
}
