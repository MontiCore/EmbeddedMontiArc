/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class LatencySymbol extends CommonSymbol{

  public static final LatencyKind KIND = LatencyKind.INSTANCE;
  private AlternativeInput latency;

  public LatencySymbol(String name, AlternativeInput latency) {
    super(name, KIND);
    this.latency = latency;
  }

  public AlternativeInput getLatency() {
    return latency;
  }
}
