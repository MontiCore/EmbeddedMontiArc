/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.montisim.simlang._symboltable;

import de.monticore.lang.montisim.util.types.AlternativeInput;
import de.monticore.symboltable.CommonSymbol;

public class PedestrianDensitySymbol extends CommonSymbol{

  public static final PedestrianDensityKind KIND = PedestrianDensityKind.INSTANCE;
  private AlternativeInput pedestrianDensity;

  public PedestrianDensitySymbol(String name, AlternativeInput pedestrianDensity) {
    super(name, KIND);
    this.pedestrianDensity = pedestrianDensity;
  }

  public AlternativeInput getPedestrianDensity() {
    return pedestrianDensity;
  }
}
