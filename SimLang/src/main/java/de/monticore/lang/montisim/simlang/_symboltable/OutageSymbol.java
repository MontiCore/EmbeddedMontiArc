/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class OutageSymbol extends CommonSymbol{

  public static final OutageKind KIND = OutageKind.INSTANCE;
  private AlternativeInput outage;

  public OutageSymbol(String name, AlternativeInput outage) {
    super(name, KIND);
    this.outage = outage;
  }

  public AlternativeInput getOutage() {
    return outage;
  }
}
